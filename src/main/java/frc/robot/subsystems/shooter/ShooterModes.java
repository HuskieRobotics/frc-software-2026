package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

public class ShooterModes extends SubsystemBase {

  // constant to configure for when running practice matches with no game data
  public final boolean WON_AUTO_PRACTICE_MATCH = true;
  private static final double SHOOT_TIME_OFFSET_SECONDS = 2.0; // offset for ball travel time to hub

  private final SwerveDrivetrain drivetrain;
  private final Shooter shooter;

  private boolean hubActive;

  /*
  Create interpolating tree map for data points
  First tree map is for our shoot on the move mode
  Second tree map is for our passing mode
  */

  private final InterpolatingDoubleTreeMap hubDistanceToVelocityMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hubDistanceToHoodMap = new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap passDistanceToVelocityMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap passDistanceToHoodMap = new InterpolatingDoubleTreeMap();

  // Shooter Mode Triggers
  private Trigger nearTrenchTrigger;
  private Trigger passModeTrigger;
  private Trigger collectAndHoldTrigger;
  private Trigger shootOTMTrigger;
  private Trigger canShootTrigger;

  // the primary mode and secondary mode will act unilaterally except for when NEAR_TRENCH
  private ShooterMode primaryMode;
  private ShooterMode secondaryMode;

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trench zone
    PASS, // passing mode,
  }

  public ShooterModes(SwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hubActive = OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean();

    // FIXME: no default value for now, change tbd?
    this.primaryMode = ShooterMode.COLLECT_AND_HOLD;
    this.secondaryMode = ShooterMode.COLLECT_AND_HOLD;

    populateShootingMap();
  }

  @Override
  public void periodic() {
    // if (!timerStarted && DriverStation.isEnabled()) {
    //   timerStarted = true;
    //   matchStartTime = Timer.getFPGATimestamp();
    // }
    this.hubActive = hubActive();

    Logger.recordOutput("ShooterModes/PrimaryMode", primaryMode);
    Logger.recordOutput("ShooterModes/SecondaryMode", secondaryMode);
    Logger.recordOutput("ShooterModes/HubActive", this.hubActive);

    Logger.recordOutput("Debug/InTrench", Field2d.getInstance().inTrenchZone());
    Logger.recordOutput("Debug/InAlliance", Field2d.getInstance().inAllianceZone());

    calculateIdealShot();
  }

  private void populateShootingMap() {
    // FIXME: populate with real data points
    hubDistanceToVelocityMap.put(0.0, 0.0);

    hubDistanceToHoodMap.put(0.0, 0.0);

    passDistanceToVelocityMap.put(0.0, 0.0);

    passDistanceToHoodMap.put(0.0, 0.0);
  }

  public boolean isShootOnTheMoveEnabled() {
    return this.primaryMode == ShooterMode.SHOOT_OTM;
  }

  public boolean isCollectAndHoldEnabled() {
    return this.primaryMode == ShooterMode.COLLECT_AND_HOLD;
  }

  public boolean isNearTrenchEnabled() {
    return this.primaryMode == ShooterMode.NEAR_TRENCH;
  }

  public boolean isPassEnabled() {
    return this.primaryMode == ShooterMode.PASS;
  }

  public boolean manualShootEnabled() {
    return this.primaryMode == ShooterMode.MANUAL_SHOOT;
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub

  // The alliance will be provided as a single character representing the color of the alliance
  // whose goal will go inactive first
  // (i.e. ‘R’ = red, ‘B’ = blue). This alliance’s goal will be active in Shifts 2 and 4.

  public boolean hubActive() {
    if (!OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean()
        && !DriverStation.isFMSAttached()) {
      return false;
    }

    // When connected to the real field, this number only changes in full integer increments, and
    // always counts down.
    // When the DS is in practice mode, this number is a floating point number, and counts down.
    // When the DS is in teleop or autonomous mode, this number is a floating point number, and
    // counts up.

    double timeRemaining = DriverStation.getMatchTime();

    // figure out if time is increasing or decreasing

    // Real Field
    double timeIntoScoringShifts;
    String gameData = DriverStation.getGameSpecificMessage();

    // hub active in auto/transition period and endgame (// time < 30 = auto or endgame, time > 130
    // = transition period)

    // can add a check for DriverStation.isAutonomousEnabled but that will always fall under
    // timeRemaining < 30
    if (timeRemaining < 30 || timeRemaining > 130 /*|| DriverStation.isAutonomousEnabled() */) {
      return true;
    } else {
      timeIntoScoringShifts = 130 - timeRemaining;
    }

    if (!gameData.isEmpty()) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue is inactive first, so if we are blue alliance, we check if closer to 50 seconds
          // (2nd period)
          if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
            return (timeIntoScoringShifts + SHOOT_TIME_OFFSET_SECONDS) % 50 > 25;
          } else {
            return (timeIntoScoringShifts + SHOOT_TIME_OFFSET_SECONDS) % 50 <= 25;
          }
        case 'R':
          // Red is inactive first, so if we are red alliance, we check if closer to 50 seconds (2nd
          // period)
          if (Field2d.getInstance().getAlliance() == Alliance.Red) {
            return timeIntoScoringShifts % 50 > 25;
          } else {
            return timeIntoScoringShifts % 50 <= 25;
          }
        default:
          // This is corrupt data
          return true;
      }
    }

    return true;
  }

  private void setNormalShooterMode(ShooterMode mode) {
    if (this.primaryMode != ShooterMode.NEAR_TRENCH) {
      this.primaryMode = mode;
    }

    this.secondaryMode = mode;
  }

  /**
   * This method sets the secondary mode as the current shooter mode before we entered the trench
   * zone. It also replaces the primary mode with NEAR_TRENCH.
   */
  private void setNearTrenchActive() {
    this.secondaryMode = this.primaryMode;
    this.primaryMode = ShooterMode.NEAR_TRENCH;
  }

  /**
   * This method returns the primary mode to the secondary mode that was active before entering the
   * trench zone.
   */
  private void returnToPreviousMode() {
    this.primaryMode = this.secondaryMode;
  }

  public void configureShooterModeTriggers() {
    nearTrenchTrigger = new Trigger(() -> Field2d.getInstance().inTrenchZone());
    nearTrenchTrigger.onTrue(Commands.runOnce(this::setNearTrenchActive));
    nearTrenchTrigger.onFalse(Commands.runOnce(this::returnToPreviousMode));

    passModeTrigger =
        new Trigger(
            () ->
                OISelector.getOperatorInterface().getPassToggle().getAsBoolean()
                    && !Field2d.getInstance().inAllianceZone());

    passModeTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.PASS)));

    collectAndHoldTrigger =
        new Trigger(
            () ->
                (!OISelector.getOperatorInterface().getPassToggle().getAsBoolean()
                        && !Field2d.getInstance().inAllianceZone())
                    || (!this.hubActive && Field2d.getInstance().inAllianceZone()));

    collectAndHoldTrigger.onTrue(
        Commands.runOnce(() -> setNormalShooterMode(ShooterMode.COLLECT_AND_HOLD)));
    shootOTMTrigger =
        new Trigger(
            () ->
                Field2d.getInstance().inAllianceZone()
                    && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()
                    && !Field2d.getInstance().inTrenchZone()
                    && this.hubActive);

    shootOTMTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.SHOOT_OTM)));

    canShootTrigger =
        new Trigger(
            () ->
                !OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()
                    && Field2d.getInstance().inAllianceZone()
                    && !Field2d.getInstance().inTrenchZone()
                    && this.hubActive);

    canShootTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.MANUAL_SHOOT)));
  }

  private ChassisSpeeds getShooterFieldRelativeVelocity() {
    ChassisSpeeds drivetrainSpeeds = RobotOdometry.getInstance().getFieldRelativeSpeeds();

    // TODO: convert to shooter relative speeds with angular velocity of turret and robot_to_shooter
    // transform
    return drivetrainSpeeds;
  }

  private void calculateIdealShot() {

    if (this.primaryMode == ShooterMode.NEAR_TRENCH) {
      shooter.setIdleVelocity(); // FIXME: change to
      // shooter.setFlywheelVelocity(RotationsPerSecond.of(otmShot[0]))
      shooter.setIdleVelocity(); // FIXME: change to shooter.setHoodAngle(min angle)
      shooter.setIdleVelocity(); // FIXME: change to shooter.setTurretAngle(otmShot[2])
      shooter.setIdleVelocity(); // FIXME: change to setKickerVelocity(0);
    } else if (this.primaryMode == ShooterMode.SHOOT_OTM
        || this.primaryMode == ShooterMode.MANUAL_SHOOT) {

      Translation2d targetLandingPosition = Field2d.getInstance().getHubCenter();

      // find our distances to target in x, y and theta
      Pose2d robotPose =
          RobotOdometry.getInstance()
              .getEstimatedPose(); // FIXME: may want to add a shooter offset to this pose to get
      // the actual position of the shooter instead of the center of
      // the robot
      double deltaX = targetLandingPosition.getX() - robotPose.getX();
      double deltaY = targetLandingPosition.getY() - robotPose.getY();
      double distance = Math.hypot(deltaX, deltaY);

      double idealShotVelocity =
          this.hubDistanceToVelocityMap.get(distance); // FIXME: may need to add velocity offset
      Angle idealTurretAngle = Radians.of(Math.atan2(deltaX, deltaY));
      Angle idealHoodAngle = Degrees.of(this.hubDistanceToHoodMap.get(distance));

      Double[] otmShot =
          calculateShootOnTheMove(idealShotVelocity, idealHoodAngle, idealTurretAngle);

      if (this.primaryMode == ShooterMode.MANUAL_SHOOT) {
        shooter.setIdleVelocity(); // FIXME: change to
        // shooter.setFlywheelVelocity(RotationsPerSecond.of(idealShotVelocity))
        shooter.setIdleVelocity(); // FIXME: change to shooter.setHoodAngle(idealHoodAngle)
        shooter.setIdleVelocity(); // FIXME: change to shooter.setTurretAngle(idealTurretAngle)
      } else {
        shooter.setIdleVelocity(); // FIXME: change to
        // shooter.setFlywheelVelocity(RotationsPerSecond.of(otmShot[0]))
        shooter.setIdleVelocity(); // FIXME: change to shooter.setHoodAngle(Radians.of(otmShot[1]))
        shooter
            .setIdleVelocity(); // FIXME: change to shooter.setTurretAngle(Radians.of(otmShot[2]))
        shooter.setIdleVelocity(); // FIXME: change to setKickerVelocity(-----);
      }
    } else if (this.primaryMode == ShooterMode.PASS) {

      Translation2d targetLandingPosition = Field2d.getInstance().getHubCenter();

      // find our distances to target in x, y and theta
      Pose2d robotPose =
          RobotOdometry.getInstance()
              .getEstimatedPose(); // FIXME: may want to add a shooter offset to this pose to get
      // the actual position of the shooter instead of the center of
      // the robot
      double deltaX = targetLandingPosition.getX() - robotPose.getX();
      double deltaY = targetLandingPosition.getY() - robotPose.getY();
      double distance = Math.hypot(deltaX, deltaY);

      double idealShotVelocity =
          this.hubDistanceToVelocityMap.get(distance); // FIXME: may need to add velocity offset
      Angle idealTurretAngle = Radians.of(Math.atan2(deltaX, deltaY));
      Angle idealHoodAngle = Degrees.of(this.hubDistanceToHoodMap.get(distance));

      Double[] otmShot =
          calculateShootOnTheMove(idealShotVelocity, idealHoodAngle, idealTurretAngle);

      shooter.setIdleVelocity(); // FIXME: change to
      // shooter.setFlywheelVelocity(RotationsPerSecond.of(idealShotVelocity))
      shooter.setIdleVelocity(); // FIXME: change to shooter.setHoodAngle(idealHoodAngle)
      shooter.setIdleVelocity(); // FIXME: change to shooter.setTurretAngle(idealTurretAngle)
    }
  }

  private Double[] calculateShootOnTheMove(double v, Angle theta, Angle phi) {
    // speeds need to be field relative
    ChassisSpeeds fieldRelativeSpeeds = getShooterFieldRelativeVelocity();
    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    // v prime
    double newFlywheelVelocity =
        Math.sqrt(
            Math.pow(v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx, 2)
                + Math.pow(v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy, 2)
                + Math.pow(v * Math.sin(theta.in(Radians)), 2));

    double newHoodAngle =
        Math.atan2(
            (v * Math.sin(theta.in(Radians))),
            Math.sqrt(
                Math.pow(v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx, 2)
                    + Math.pow(
                        v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy, 2)));

    double newTurretAngle =
        Math.atan2(
            (v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy),
            (v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx));

    return new Double[] {newFlywheelVelocity, newHoodAngle, newTurretAngle};
  }
}
