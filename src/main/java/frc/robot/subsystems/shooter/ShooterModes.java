package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;

public class ShooterModes extends SubsystemBase {

  private final SwerveDrivetrain drivetrain;
  private final Shooter shooter;

  // Shooter Mode Triggers
  private Trigger nearTrenchTrigger;
  private Trigger passModeTrigger;
  private Trigger collectAndHoldTrigger;
  private Trigger shootOTMTrigger;
  private Trigger canShootTrigger;

  // the primary mode and secondary mode will act unilaterally except for when NEAR_TRENCH
  private ShooterMode primaryMode;
  private ShooterMode secondaryMode;

  private String gameData;

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trench zone
    PASS // passing mode
  }

  public ShooterModes(SwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;

    // no default value for now, change tbd?
    this.primaryMode = null;
    this.secondaryMode = null;

    configureShooterModeTriggers();
  }

  @Override
  public void periodic() {
    if (this.primaryMode != ShooterMode.NEAR_TRENCH) {
      setMode();
    }
  }

  private void setMode() {
    if (Field2d.getInstance().inAllianceZone()) {
      if (hubActive()
          && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
        setNormalShooterMode(ShooterMode.SHOOT_OTM);
      } else {
        setNormalShooterMode(ShooterMode.MANUAL_SHOOT);
      }
    } else {
      if (OISelector.getOperatorInterface()
          .getPassToggle()
          .getAsBoolean()) { // pass toggled by operator
        setNormalShooterMode(ShooterMode.PASS);
      } else {
        setNormalShooterMode(ShooterMode.COLLECT_AND_HOLD);
      }
    }
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

  public void getTrajectory() {
    if (this.primaryMode == ShooterMode.NEAR_TRENCH) {
      // set hood to max
      // model for aimed position minus hood (should be an extra redundancy check w/i that method
      // for NEAR_TRENCH)
    } else if (this.primaryMode == ShooterMode.MANUAL_SHOOT) {
      // model for aimed position
      // x stance
    } else if (this.primaryMode == ShooterMode.COLLECT_AND_HOLD) {
      // model for aimed position
    } else if (this.primaryMode == ShooterMode.SHOOT_OTM) {
      // model for aimed position
      // adjust for OTM
    } else if (this.primaryMode == ShooterMode.PASS) {
      // model for aimed position, which would be the nearest corner
      // adjust for OTM
    }
  }

  public void getTurret() {

    if (this.primaryMode == ShooterMode.MANUAL_SHOOT
        || this.primaryMode == ShooterMode.COLLECT_AND_HOLD) {
      // aim turret at hub
    } else if (this.primaryMode == ShooterMode.SHOOT_OTM) {
      // aim turret at hub
      // adjust for OTM
    } else if (this.primaryMode == ShooterMode.PASS) {
      // aim turret at nearest corner
      // adjust for OTM
    }
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub
  public boolean hubActive() {
    return true;
  }

  private void setNormalShooterMode(ShooterMode mode) {
    this.primaryMode = mode;
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

  private void configureShooterModeTriggers() {
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
                !OISelector.getOperatorInterface().getPassToggle().getAsBoolean()
                    && !Field2d.getInstance().inAllianceZone());

    collectAndHoldTrigger.onTrue(
        Commands.runOnce(() -> setNormalShooterMode(ShooterMode.COLLECT_AND_HOLD)));

    shootOTMTrigger =
        new Trigger(
            () ->
                Field2d.getInstance().inAllianceZone()
                    && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()
                    && !isNearTrenchEnabled()
                    && hubActive());

    shootOTMTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.SHOOT_OTM)));

    canShootTrigger =
        new Trigger(
            () ->
                !OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()
                    && Field2d.getInstance().inAllianceZone()
                    && !isNearTrenchEnabled()
                    && hubActive());

    canShootTrigger.onTrue(Commands.runOnce(() -> setNormalShooterMode(ShooterMode.MANUAL_SHOOT)));
  }

  private ChassisSpeeds getShooterFieldRelativeVelocity() {
    ChassisSpeeds drivetrainSpeeds = RobotOdometry.getInstance().getFieldRelativeSpeeds();

    // TODO: convert to shooter relative speeds with angular velocity of turret and robot_to_shooter
    // transform
    return drivetrainSpeeds;
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
        Math.atan(
            (v * Math.sin(theta.in(Radians)))
                / Math.sqrt(
                    Math.pow(
                            v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx,
                            2)
                        + Math.pow(
                            v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy,
                            2)));

    double newTurretAngle =
        Math.atan(
            (v * Math.cos(theta.in(Radians)) * Math.sin(phi.in(Radians)) - robotVy)
                / (v * Math.cos(theta.in(Radians)) * Math.cos(phi.in(Radians)) - robotVx));

    return new Double[] {newFlywheelVelocity, newHoodAngle, newTurretAngle};
  }
}
