package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.LoggedTunableNumber;
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
  private Trigger lockShooterTrigger;
  private Trigger turretLockNZTrigger;
  private Trigger testingModeTrigger;

  private boolean turretAutoLocked = false;

  private final LoggedTunableNumber testingFlywheelVelocity =
      new LoggedTunableNumber("ShooterModes/Testing/FlywheelVelocityRPS", 0.0);
  private final LoggedTunableNumber testingHoodAngle =
      new LoggedTunableNumber("ShooterModes/Testing/HoodAngleDegrees", HOOD_MIN_ANGLE.in(Degrees));
  private final LoggedTunableNumber testingTurretAngle =
      new LoggedTunableNumber("ShooterModes/Testing/TurretAngleDegrees", 0.0);

  // the primary mode and secondary mode will act unilaterally except for when NEAR_TRENCH
  private ShooterMode primaryMode;
  private ShooterMode secondaryMode;

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trench zone
    PASS, // passing mode,
    SHOOTER_LOCKED, // set our manual hood, turret, and flywheel values for money shot
    TESTING // testing mode for testing
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
    this.hubActive = hubActive();

    Logger.recordOutput("ShooterModes/PrimaryMode", primaryMode);
    Logger.recordOutput("ShooterModes/SecondaryMode", secondaryMode);
    Logger.recordOutput("ShooterModes/HubActive", this.hubActive);

    calculateIdealShot();
  }

  private void populateShootingMap() {
    // FIXME: populate with real data points
    hubDistanceToVelocityMap.put(0.0, 0.0);

    hubDistanceToHoodMap.put(0.0, 0.0);

    passDistanceToVelocityMap.put(0.0, 0.0);

    passDistanceToHoodMap.put(0.0, 0.0);
  }

  private void setTurretAutoLocked(boolean locked) {
    this.turretAutoLocked = locked;
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

  public boolean isLockedShooterEnabled() {
    return this.primaryMode == ShooterMode.SHOOTER_LOCKED;
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
    if (timeRemaining < (30 + SHOOT_TIME_OFFSET_SECONDS)
        || timeRemaining
            > (130 + SHOOT_TIME_OFFSET_SECONDS) /*|| DriverStation.isAutonomousEnabled() */) {
      return true;
    } else {
      // Always look SHOOT_TIME_OFFSET_SECONDS into the future to account for the time it takes for
      // the fuel to reach the hub. We need to stop shooting SHOOT_TIME_OFFSET_SECONDS seconds
      // before the end of the period to ensure that the fuel we shoot before the end of the period
      // actually counts. Similarly, we can start shooting SHOOT_TIME_OFFSET_SECONDS seconds before
      // the start of the next active period because the fuel we shoot at the start of the period
      // will count as long as it doesn't reach the hub before the start of the period.
      timeIntoScoringShifts = 130 - timeRemaining + SHOOT_TIME_OFFSET_SECONDS;
    }

    if (!gameData.isEmpty()) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue is inactive first, so if we are blue alliance, we check if closer to 50 seconds
          // (2nd period)
          if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
            return timeIntoScoringShifts % 50 > 25;
          } else {
            return timeIntoScoringShifts % 50 <= 25;
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
    if (this.primaryMode != ShooterMode.NEAR_TRENCH
        && this.primaryMode != ShooterMode.SHOOTER_LOCKED
        && this.primaryMode != ShooterMode.TESTING) {
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
                    && !Field2d.getInstance().inAllianceZone()
                    && !Field2d.getInstance().inTrenchZone());

    passModeTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.PASS)));

    collectAndHoldTrigger =
        new Trigger(
            () ->
                (!OISelector.getOperatorInterface().getPassToggle().getAsBoolean()
                        && !Field2d.getInstance().inAllianceZone()
                        && !Field2d.getInstance().inTrenchZone())
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
                    && !OISelector.getOperatorInterface().getLockShooterToggle().getAsBoolean()
                    && Field2d.getInstance().inAllianceZone()
                    && !Field2d.getInstance().inTrenchZone()
                    && this.hubActive);

    canShootTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.MANUAL_SHOOT)));

    turretLockNZTrigger =
        new Trigger(
            () ->
                OISelector.getOperatorInterface().getLockTurretForBankToggle().getAsBoolean()
                    || ((!Field2d.getInstance().inAllianceZone()
                            && !Field2d.getInstance().inTrenchZone())
                        && this.primaryMode != ShooterMode.PASS));

    turretLockNZTrigger.onTrue(Commands.runOnce(() -> setTurretAutoLocked(true)));
    turretLockNZTrigger.onFalse(Commands.runOnce(() -> setTurretAutoLocked(false)));

    lockShooterTrigger =
        new Trigger(() -> OISelector.getOperatorInterface().getLockShooterToggle().getAsBoolean());
    lockShooterTrigger.onTrue(
        Commands.runOnce(() -> setNormalShooterMode(ShooterMode.SHOOTER_LOCKED)));

    testingModeTrigger =
        new Trigger(
            () -> OISelector.getOperatorInterface().getShooterModesTestingToggle().getAsBoolean());
    testingModeTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.TESTING)));
  }

  private ChassisSpeeds getShooterFieldRelativeVelocity() {
    ChassisSpeeds drivetrainSpeeds = RobotOdometry.getInstance().getFieldRelativeSpeeds();

    Pose2d robotPose = RobotOdometry.getInstance().getEstimatedPose();

    double robotVelocity = drivetrainSpeeds.vxMetersPerSecond;

    double deltaXFieldRelativeTurret =
        robotPose.getX() * ROBOT_TO_TURRET_TRANSFORM.getX()
            - robotPose.getY() * ROBOT_TO_TURRET_TRANSFORM.getY();
    double deltaYFieldRelativeTurret =
        robotPose.getX() * ROBOT_TO_TURRET_TRANSFORM.getY()
            + robotPose.getY() * ROBOT_TO_TURRET_TRANSFORM.getX();

    double tangentialVelocityTurretX =
        -(drivetrainSpeeds.omegaRadiansPerSecond * deltaYFieldRelativeTurret);
    double tangentialVelocityTurretY =
        (drivetrainSpeeds.omegaRadiansPerSecond * deltaXFieldRelativeTurret);

    return new ChassisSpeeds(
        robotVelocity + tangentialVelocityTurretX,
        robotVelocity + tangentialVelocityTurretY,
        drivetrainSpeeds.omegaRadiansPerSecond); // FIXME: fix this
  }

  private void calculateIdealShot() {

    if (this.primaryMode == ShooterMode.TESTING) {
      shooter.setFlywheelVelocity(RotationsPerSecond.of(testingFlywheelVelocity.get()));
      shooter.setHoodPosition(Degrees.of(testingHoodAngle.get()));
      shooter.setTurretPosition(Degrees.of(testingTurretAngle.get()));
      return;
    }

    boolean lockToggleOn =
        OISelector.getOperatorInterface().getLockTurretForBankToggle().getAsBoolean();

    if (((this.turretAutoLocked && this.primaryMode != ShooterMode.PASS)
            || (lockToggleOn && this.primaryMode != ShooterMode.PASS))
        && this.primaryMode != ShooterMode.SHOOTER_LOCKED) {
      shooter.setTurretPosition(TURRET_LOCK_POSITION_DEGREES);
    }

    if (this.primaryMode == ShooterMode.NEAR_TRENCH) {
      Translation2d targetLandingPosition = Field2d.getInstance().getHubCenter();

      // get static shot setpoints in rps, degrees, degrees
      Double[] staticShotSetpoints = getIdealStaticShotSetpoints(targetLandingPosition);

      if (!OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
        shooter.setFlywheelVelocity(RotationsPerSecond.of(staticShotSetpoints[0]));
        shooter.setHoodPosition(HOOD_LOWER_ANGLE_LIMIT);
        shooter.setTurretPosition(Degrees.of(staticShotSetpoints[2]));
      } else {
        // get shoot on the move set points in rps, degrees, degrees
        Double[] otmShot =
            calculateShootOnTheMove(
                staticShotSetpoints[0],
                Degrees.of(staticShotSetpoints[1]),
                Degrees.of(staticShotSetpoints[2]));

        shooter.setFlywheelVelocity(RotationsPerSecond.of(otmShot[0]));
        shooter.setHoodPosition(HOOD_LOWER_ANGLE_LIMIT);
        shooter.setTurretPosition(Degrees.of(otmShot[2]));
      }
    } else if (this.primaryMode == ShooterMode.SHOOTER_LOCKED) {
      shooter.setFlywheelVelocity(LOCK_SHOT_FLYWHEEL_RPS);
      shooter.setHoodPosition(LOCK_SHOT_HOOD_ANGLE);
      shooter.setTurretPosition(LOCK_SHOT_TURRET_ANGLE);
    } else if (this.primaryMode == ShooterMode.SHOOT_OTM
        || this.primaryMode == ShooterMode.MANUAL_SHOOT
        || this.primaryMode == ShooterMode.COLLECT_AND_HOLD) {

      Translation2d targetLandingPosition = Field2d.getInstance().getHubCenter();
      Double[] staticShotSetpoints = getIdealStaticShotSetpoints(targetLandingPosition);

      if (!OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()
          || this.primaryMode == ShooterMode.MANUAL_SHOOT) {
        shooter.setFlywheelVelocity(RotationsPerSecond.of(staticShotSetpoints[0]));
        shooter.setHoodPosition(Degrees.of(staticShotSetpoints[1]));
        shooter.setTurretPosition(Degrees.of(staticShotSetpoints[2]));
      } else {
        Double[] otmShot =
            calculateShootOnTheMove(
                staticShotSetpoints[0],
                Degrees.of(staticShotSetpoints[1]),
                Degrees.of(staticShotSetpoints[2]));

        shooter.setFlywheelVelocity(RotationsPerSecond.of(otmShot[0]));
        shooter.setHoodPosition(Degrees.of(otmShot[1]));
        shooter.setTurretPosition(Degrees.of(otmShot[2]));
      }
    } else if (this.primaryMode == ShooterMode.PASS) {

      if (Field2d.getInstance().inOpponentAllianceZone()
          && drivetrain.getPose().getY() < FieldConstants.Hub.leftFace.getY()
          && drivetrain.getPose().getY() > FieldConstants.Hub.rightFace.getY()) {

        shooter.setHoodPosition(HOOD_LOWER_ANGLE_LIMIT);
        shooter.setFlywheelVelocity(FLYWHEEL_PASS_OVER_NET_VELOCITY);
      } else {
        Translation2d targetLandingPosition =
            Field2d.getInstance().getNearestPassingZone().getTranslation();

        Double[] idealShotSetpoints = getIdealPassSetpoints(targetLandingPosition);
        Double[] otmShot =
            calculateShootOnTheMove(
                idealShotSetpoints[0],
                Degrees.of(idealShotSetpoints[1]),
                Degrees.of(idealShotSetpoints[2]));

        double PASSING_TOLERANCE = 14; // meters

        if (Math.sqrt(
                Math.pow(targetLandingPosition.getY(), 2)
                    + Math.pow(targetLandingPosition.getX(), 2))
            < PASSING_TOLERANCE) {
          shooter.setFlywheelVelocity(FLYWHEEL_MAX_VELOCITY_RPS);
          shooter.setHoodPosition(HOOD_MAX_PASSING_ANGLE);
          shooter.setTurretPosition(Degrees.of(otmShot[2]));
        } else {
          shooter.setFlywheelVelocity(RotationsPerSecond.of(otmShot[0]));
          shooter.setHoodPosition(Degrees.of(otmShot[1]));
          shooter.setTurretPosition(Degrees.of(otmShot[2]));
        }
      }
    }
  }

  private Double[] calculateShootOnTheMove(double v, Angle theta, Angle phi) {
    // speeds need to be field relative
    ChassisSpeeds fieldRelativeSpeeds = getShooterFieldRelativeVelocity();
    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    double newFlywheelVelocity =
        Math.sqrt(
            Math.pow(v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx, 2)
                + Math.pow(v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy, 2)
                + Math.pow(v * Math.sin(theta.in(Radians)), 2));

    // angles are converted to degrees
    double newHoodAngle =
        (180.0 / Math.PI)
            * Math.atan2(
                Math.sqrt(
                    Math.pow(
                            v * Math.sin(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx,
                            2)
                        + Math.pow(
                            v * Math.sin(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy,
                            2)),
                v * Math.cos(theta.in(Radians)));

    double newTurretAngle =
        (180.0 / Math.PI)
            * Math.atan2(
                (v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy),
                (v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx));

    return new Double[] {newFlywheelVelocity, newHoodAngle, newTurretAngle};
  }

  private double idealVelocityFromFunction(double distance) {
    double vMetersPerSecond =
        0.0094236446 * Math.pow(distance, 3)
            - 0.1282900256 * Math.pow(distance, 2)
            + 1.3314733073 * distance
            + 4.0571728302;

    // find average circumference of final contact points wheels radius 1.0 and 2.0
    double avgCircumference = 2 * Math.PI * ((2.0 + 1.0) / 2.0) * 0.0254;

    return vMetersPerSecond / avgCircumference; // convert to RPS
  }

  private Angle idealHoodAngleFromFunction(double distance) {
    // The original function finds angle in degrees from vertical rather than horizontal
    // so we need to subtract from 90 degrees to get the angle from horizontal
    return Degrees.of(
        -0.0826994753 * Math.pow(distance, 3)
            + 0.6567904524 * Math.pow(distance, 2)
            + 0.6209511280 * distance
            + 21.6348611545);
  }

  private Double[] getIdealStaticShotSetpoints(Translation2d targetLandingPosition) {
    // find our distances to target in x, y and theta

    // transform robot pose by calculated robot to shooter transform
    Pose2d robotPose =
        RobotOdometry.getInstance().getEstimatedPose().transformBy(ROBOT_TO_TURRET_TRANSFORM);

    double deltaX = targetLandingPosition.getX() - (robotPose.getX());
    double deltaY = targetLandingPosition.getY() - (robotPose.getY());
    double distance = Math.hypot(deltaY, deltaX);

    double idealShotVelocity =
        idealVelocityFromFunction(distance)
            * FLYWHEEL_VELOCITY_SCALE_FACTOR; // this.hubDistanceToVelocityMap.get(distance);

    Angle fieldRelativeTurretAngle = Radians.of(Math.atan2(deltaY, deltaX));
    Rotation2d robotRelativeTurretAngleRadians =
        new Rotation2d(fieldRelativeTurretAngle).minus(robotPose.getRotation());
    Angle robotRelativeTurretAngle = Degrees.of(robotRelativeTurretAngleRadians.getDegrees());

    Angle idealHoodAngle =
        idealHoodAngleFromFunction(distance)
            .plus(
                HOOD_OFFSET_WHEN_SHOOTING); // Degrees.of(this.hubDistanceToHoodMap.get(distance));

    return new Double[] {
      idealShotVelocity, idealHoodAngle.in(Degrees), robotRelativeTurretAngle.in(Degrees)
    };
  }

  private Double[] getIdealPassSetpoints(Translation2d targetLandingPosition) {
    Pose2d robotPose =
        RobotOdometry.getInstance().getEstimatedPose().transformBy(ROBOT_TO_TURRET_TRANSFORM);

    double deltaX = targetLandingPosition.getX() - robotPose.getX();
    double deltaY = targetLandingPosition.getY() - robotPose.getY();
    double distance = Math.hypot(deltaY, deltaX);

    // function only applies for hub shots but added a constant value to differentiate in sim
    double idealShotVelocity =
        idealVelocityFromFunction(distance)
            * FLYWHEEL_VELOCITY_SCALE_FACTOR; // this.passDistanceToVelocityMap.get(distance);

    Angle fieldRelativeTurretAngle = Radians.of(Math.atan2(deltaY, deltaX));
    Rotation2d robotRelativeTurretAngleRadians =
        robotPose.getRotation().minus(new Rotation2d(fieldRelativeTurretAngle));
    Angle robotRelativeTurretAngle = Degrees.of(robotRelativeTurretAngleRadians.getDegrees());

    Angle idealHoodAngle =
        idealHoodAngleFromFunction(distance)
            .plus(
                HOOD_OFFSET_WHEN_SHOOTING); // Degrees.of(this.passDistanceToHoodMap.get(distance));

    return new Double[] {
      idealShotVelocity, idealHoodAngle.in(Degrees), robotRelativeTurretAngle.in(Degrees)
    };
  }
}
