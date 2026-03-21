package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.MathUtils;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.commands.AutonomousCommandsFactory;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

public class ShooterModes extends SubsystemBase {

  // Stop shooting 0.75 seconds before the end of the period to account for the time it takes for
  // the fuel to reach the hub (approximately 1.25s) and ensure the fuel is processed within the 3
  // second window after the end of the period (almost all fuel is processed within 2.5 seconds)
  private static final double STOP_SHOOTING_TIME_OFFSET_SECONDS = 0.75;

  // Start shooting 1.75 seconds before the start of the period to account for the time it takes for
  // the fuel to reach the hub (approximately 1.25s) and ensure the fuel isn't processed until the
  // period starts (almost no fuel is processed before 0.5 seconds elapse)
  private static final double START_SHOOTING_TIME_OFFSET_SECONDS = 1.75;

  private static final double END_OF_SHIFT_WARNING_SECONDS =
      5.0; // time before the end of the shift to flash the LE#Ds

  private final Shooter shooter;

  private boolean hubActive;
  private double shotVelocityMultiplier = 1.0;
  private double turretAngleAdjustmentDeg = 0.0;

  private Timer turretOutsideSetpointTimer = new Timer();
  private Timer turretUnJammingTimer = new Timer();

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

  private final LoggedTunableNumber testingEnable =
      new LoggedTunableNumber("ShooterModes/Testing/EnableTesting", 0.0);
  private final LoggedTunableNumber testingFlywheelVelocity =
      new LoggedTunableNumber("ShooterModes/Testing/FlywheelVelocityRPS", 0.0);
  private final LoggedTunableNumber testingHoodAngle =
      new LoggedTunableNumber(
          "ShooterModes/Testing/HoodAngleDegrees", Units.rotationsToDegrees(HOOD_MIN_ANGLE_ROT));
  private final LoggedTunableNumber testingTurretAngle =
      new LoggedTunableNumber("ShooterModes/Testing/TurretAngleDegrees", 0.0);

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trench zone
    MANUAL_PASS, // passing mode,
    PASS_OTM, // passing on the move
    SHOOTER_LOCKED, // set our manual hood, turret, and flywheel values for money shot
    TESTING // testing mode for testing
  }

  private class ShooterSetpoints {
    double flywheelVelocityRPS;
    double hoodAngleRot;
    double turretAngleRot;

    public ShooterSetpoints(
        double flywheelVelocityRPS, double hoodAngleRot, double turretAngleRot) {
      this.flywheelVelocityRPS = flywheelVelocityRPS;
      this.hoodAngleRot = hoodAngleRot;
      this.turretAngleRot = turretAngleRot;
    }
  }

  private ShooterMode currentMode = ShooterMode.COLLECT_AND_HOLD;

  public ShooterModes(Shooter shooter) {
    this.shooter = shooter;
    this.hubActive = OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean();

    populateMaps();
  }

  @Override
  public void periodic() {
    this.hubActive = isSafeToShootInHub();

    determineModeAndSetShooter();

    if (!isTurretNotNearSetPoint()) {
      turretOutsideSetpointTimer.restart();
    }

    Logger.recordOutput("ShooterModes/Shot Multiplier", this.shotVelocityMultiplier);
    Logger.recordOutput(
        "ShooterModes/Turret Angle Adjustment Degrees", this.turretAngleAdjustmentDeg);
    Logger.recordOutput("ShooterModes/CurrentMode", this.currentMode);
    Logger.recordOutput("ShooterModes/HubActive", this.hubActive);
    Logger.recordOutput("ShooterModes/turret not near setpoint", isTurretNotNearSetPoint());
    Logger.recordOutput(
        "ShooterModes/Turret Outside Setpoint Time Elapsed", turretOutsideSetpointTimer.get());
    Logger.recordOutput("ShooterModes/Turret Unjamming Time Elapsed", turretUnJammingTimer.get());
  }

  private void populateMaps() {
    // Velocity Shooting into Hub Map
    hubDistanceToVelocityMap.put(1.8034, 28.0);
    hubDistanceToVelocityMap.put(2.032, 29.0);
    hubDistanceToVelocityMap.put(2.413, 30.0);
    hubDistanceToVelocityMap.put(2.8702, 32.0);
    hubDistanceToVelocityMap.put(2.9972, 33.0);
    hubDistanceToVelocityMap.put(3.2512, 34.0);
    hubDistanceToVelocityMap.put(3.68, 35.0);
    hubDistanceToVelocityMap.put(3.83, 35.0);
    hubDistanceToVelocityMap.put(4.4196, 37.5);
    hubDistanceToVelocityMap.put(4.80, 38.0);
    hubDistanceToVelocityMap.put(5.14, 39.0);
    // hubDistanceToVelocityMap.put(5.38, 41.0);

    // hubDistanceToVelocityMap.put(6.22, 42.0);

    // Hood Angle Shooting into Hub Map
    hubDistanceToHoodMap.put(1.8034, 22.0);
    hubDistanceToHoodMap.put(2.032, 22.0);
    hubDistanceToHoodMap.put(2.413, 22.0);
    hubDistanceToHoodMap.put(2.8702, 23.0);
    hubDistanceToHoodMap.put(2.9972, 23.0);
    hubDistanceToHoodMap.put(3.2512, 25.0);
    hubDistanceToHoodMap.put(3.68, 26.0);
    hubDistanceToHoodMap.put(3.83, 26.0);
    hubDistanceToHoodMap.put(4.4196, 27.0);
    hubDistanceToHoodMap.put(4.80, 28.0);
    hubDistanceToHoodMap.put(5.14, 29.0);
    // hubDistanceToHoodMap.put(5.38, 28.0);
    hubDistanceToHoodMap.put(6.22, 29.0);

    // Velocity Passing Map
    passDistanceToVelocityMap.put(3.9, 30.0);
    passDistanceToVelocityMap.put(5.05, 34.0);
    passDistanceToVelocityMap.put(6.45, 38.0);
    passDistanceToVelocityMap.put(8.07, 43.0);
    passDistanceToVelocityMap.put(9.55, 46.0);
    passDistanceToVelocityMap.put(11.05, 51.0);
    passDistanceToVelocityMap.put(13.0, 57.0);
    passDistanceToVelocityMap.put(14.0, 60.0);

    // Hood Angle Passing Map
    passDistanceToHoodMap.put(3.9, 29.0);
    passDistanceToHoodMap.put(5.05, 30.0);
    passDistanceToHoodMap.put(6.45, 33.0);
    passDistanceToHoodMap.put(8.07, 36.0);
    passDistanceToHoodMap.put(9.55, 39.0);
    passDistanceToHoodMap.put(11.05, 42.0);
    passDistanceToHoodMap.put(13.0, 46.0);
    passDistanceToHoodMap.put(14.0, 49.0);
  }

  public boolean isShootOnTheMoveEnabled() {
    return this.currentMode == ShooterMode.SHOOT_OTM;
  }

  public boolean isCollectAndHoldEnabled() {
    return this.currentMode == ShooterMode.COLLECT_AND_HOLD;
  }

  public boolean isNearTrenchEnabled() {
    return this.currentMode == ShooterMode.NEAR_TRENCH;
  }

  public boolean isManualPassEnabled() {
    return this.currentMode == ShooterMode.MANUAL_PASS;
  }

  public boolean isPassOnTheMoveEnabled() {
    return this.currentMode == ShooterMode.PASS_OTM;
  }

  public boolean isManualShootEnabled() {
    return this.currentMode == ShooterMode.MANUAL_SHOOT;
  }

  public boolean isLockedShooterEnabled() {
    return this.currentMode == ShooterMode.SHOOTER_LOCKED;
  }

  public boolean isTurretNotNearSetPoint() {
    double threshold = TURRET_DISTANCE_TO_SETPOINT_THRESHOLD_WHEN_SHOOTING_ROT;
    if (currentMode == ShooterMode.PASS_OTM || currentMode == ShooterMode.MANUAL_PASS) {
      threshold = TURRET_DISTANCE_TO_SETPOINT_THRESHOLD_WHEN_PASSING_ROT;
    }
    return !MathUtils.isNear(
        shooter.getTurretPositionRot(), shooter.getTurretReferencePositionRot(), threshold);
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub

  // The alliance will be provided as a single character representing the color of the alliance
  // whose goal will go inactive first
  // (i.e. ‘R’ = red, ‘B’ = blue). This alliance’s goal will be active in Shifts 2 and 4.

  public boolean isSafeToShootInHub() {
    if (!DriverStation.isFMSAttached()) {
      return OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean();
    }

    // The value will count down the time remaining in the current period (auto or teleop).
    // When connected to the real field, this number only changes in full integer increments, and
    //  always counts down.
    // When the DS is in practice mode, this number is a floating point number, and counts down.
    // When the DS is in teleop or autonomous mode, this number is a floating point number, and
    //  counts up.

    double timeRemaining = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // Auto (20 s) - always safe to shoot and don't need to stop early
    if (timeRemaining < 20) {
      return true;
    }

    // if we are in shift 1 - 4, give a warning on the LEDs when we are within the
    // END_OF_SHIFT_WARNING_SECONDS before the end of the shift
    if (timeRemaining < 130 && timeRemaining > 30) {
      if ((timeRemaining - 30) % 25 < END_OF_SHIFT_WARNING_SECONDS) {
        LEDs.getInstance().requestState(LEDs.States.END_OF_PERIOD);
      }
    }

    boolean isInactiveFirst = false;
    if (!gameData.isEmpty()) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue is inactive first
          isInactiveFirst = Field2d.getInstance().getAlliance() == Alliance.Blue;
          break;
        case 'R':
          // Red is inactive first
          isInactiveFirst = Field2d.getInstance().getAlliance() == Alliance.Red;
          break;
        default:
          // This is corrupt data
          return true;
      }
    }

    if (isInactiveFirst) {
      // Transition - safe to shoot but need to stop early
      if (timeRemaining > 130 + STOP_SHOOTING_TIME_OFFSET_SECONDS) {
        return true;
      }
      // Shift 1 - not safe to shoot but start shooting before shift 2 starts
      else if (timeRemaining > 105 + START_SHOOTING_TIME_OFFSET_SECONDS) {
        return false;
      }
      // Shift 2 - safe to shoot but need to stop early
      else if (timeRemaining > 80 + STOP_SHOOTING_TIME_OFFSET_SECONDS) {
        return true;
      }
      // Shift 3 - not safe to shoot but start shooting before shift 4 starts
      else if (timeRemaining > 55 + START_SHOOTING_TIME_OFFSET_SECONDS) {
        return false;
      }
      // Shift 4 and End Game - safe to shoot
      else {
        return true;
      }
    } else {
      // Transition - safe to shoot and don't need to stop early
      if (timeRemaining > 130) {
        return true;
      }
      // Shift 1 - safe to shoot but need to stop early
      else if (timeRemaining > 105 + STOP_SHOOTING_TIME_OFFSET_SECONDS) {
        return true;
      }
      // Shift 2 - not safe to shoot but start shooting before shift 3 starts
      else if (timeRemaining > 80 + START_SHOOTING_TIME_OFFSET_SECONDS) {
        return false;
      }
      // Shift 3 - safe to shoot but need to stop early
      else if (timeRemaining > 55 + STOP_SHOOTING_TIME_OFFSET_SECONDS) {
        return true;
      }
      // Shift 4 - not safe to shoot but start shooting before end game starts
      else if (timeRemaining > 30 + START_SHOOTING_TIME_OFFSET_SECONDS) {
        return false;
      }
      // End Game - safe to shoot and don't need to stop early
      else {
        return true;
      }
    }
  }

  private ChassisSpeeds getShooterFieldRelativeVelocity() {

    ChassisSpeeds drivetrainSpeeds = RobotOdometry.getInstance().getFieldRelativeSpeeds();
    Pose2d robotPose = RobotOdometry.getInstance().getEstimatedPose();

    double robotHeading = robotPose.getRotation().getRadians();
    double xs = ROBOT_TO_TURRET_TRANSFORM.getX();
    double ys = ROBOT_TO_TURRET_TRANSFORM.getY();

    double xFieldTurret = Math.cos(robotHeading) * xs - Math.sin(robotHeading) * ys;
    double yFieldTurret = Math.sin(robotHeading) * xs + Math.cos(robotHeading) * ys;

    double tangentialVelocityTurretX = -yFieldTurret * drivetrainSpeeds.omegaRadiansPerSecond;
    double tangentialVelocityTurretY = xFieldTurret * drivetrainSpeeds.omegaRadiansPerSecond;

    return new ChassisSpeeds(
        drivetrainSpeeds.vxMetersPerSecond + tangentialVelocityTurretX,
        drivetrainSpeeds.vyMetersPerSecond + tangentialVelocityTurretY,
        drivetrainSpeeds.omegaRadiansPerSecond);
  }

  /**
   * FIXME: update to reflect refactored approach
   *
   * <p>Conditions for triggering the various shooter modes:
   *
   * <p>NEAR_TRENCH: when the robot enters the trench zone, it will always switch to the NEAR_TRENCH
   * mode. When the robot leaves the trench zone, it should return to the previous mode.
   *
   * <p>PASS: when the pass toggle is on and the robot is not in the alliance zone or trench zone,
   * it switches to PASS mode. The robot stays in PASS mode until another trigger occurs.
   *
   * <p>SHOOT_OTM: when the shoot on the move toggle is on, the robot is in the alliance zone, not
   * in the trench zone, and the hub is active, it switches to SHOOT_OTM mode. The robot stays in
   * SHOOT_OTM mode until another trigger occurs.
   *
   * <p>MANUAL_SHOOT: when the shoot on the move toggle is off, the lock shooter toggle is off, the
   * robot is in the alliance zone, not in the trench zone, and the hub is active, it switches to
   * MANUAL_SHOOT mode. The robot stays in MANUAL_SHOOT mode until another trigger occurs.
   *
   * <p>These aren't shooter modes but other relevant conditions that are set by triggers:
   *
   * <p>turret auto locked: when the lock turret for bank toggle is on, or when we are not in the
   * alliance zone or trench zone and not in PASS mode, the turret will be auto locked to a specific
   * angle to prepare for the money shot. The turret will be locked until the toggle is turned off
   * or we enter PASS mode or we enter our alliance zone.
   *
   * <p>Exceptional conditions (i.e., manual overrides)
   *
   * <p>TESTING: when the testing mode toggle is on, it switches to TESTING mode, which allows us to
   * set the hood angle, flywheel velocity, and turret angle to specific values for testing
   * purposes. The robot stays in TESTING mode until the toggle is turned off.
   *
   * <p>SHOOTER_LOCKED: when the lock shooter toggle is on, it switches to SHOOTER_LOCKED mode,
   * which sets the hood angle, flywheel velocity, and turret angle to specific values for a money
   * shot. The robot stays in SHOOTER_LOCKED mode until the toggle is turned off.
   */
  private void determineModeAndSetShooter() {

    // check for testing mode first since that overrides all other modes and doesn't rely on any
    // conditions except for the toggle
    if (testingEnable.get() == 1) {
      this.currentMode = ShooterMode.TESTING;
      shooter.setFlywheelVelocity(testingFlywheelVelocity.get());
      shooter.setHoodPosition(Units.degreesToRotations(testingHoodAngle.get()));
      shooter.setTurretPosition(Units.degreesToRotations(testingTurretAngle.get()));
      return;
    }

    // check if the shooter system test is running; if so, don't update the shooter setpoints so the
    // test can control the shooter without interference from the shooter modes logic
    if (!DriverStation.isFMSAttached() && shooter.isSystemTestRunning()) {
      return;
    }

    // determine if the robot will be potentially shooting or passing, which is based on  whether we
    // are in the alliance zone or not

    ShooterSetpoints shooterSetpoints;
    Translation2d targetLandingPosition;

    if (Field2d.getInstance().inAllianceZone()) {
      // assume that if the robot is shooting from rest and adjust later as needed
      targetLandingPosition = Field2d.getInstance().getHubCenter();
      shooterSetpoints =
          getIdealStaticSetpoints(
              targetLandingPosition, hubDistanceToVelocityMap, hubDistanceToHoodMap, true);

      // if the hub is not active, put the robot in collect and hold mode to prepare for when the
      // hub becomes active
      if (!this.hubActive || Field2d.getInstance().inTowerNoPassZone()) {
        this.currentMode = ShooterMode.COLLECT_AND_HOLD;
      } else {
        // if the hub is active, the robot is either shooting on the move or manually shooting based
        // on the shoot on the move toggle
        if (OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
          this.currentMode = ShooterMode.SHOOT_OTM;

          // update the setpoints based on the robots velocity for shoot on the move
          shooterSetpoints = calculateShootOnTheMove(shooterSetpoints);
        } else {
          this.currentMode = ShooterMode.MANUAL_SHOOT;

          if (shooter.isFlywheelAtSetPoint()
              && shooter.isHoodAtSetPoint()
              && shooter.isTurretAtSetPoint()) {
            LEDs.getInstance().requestState(LEDs.States.READY_TO_SHOOT);
          }
        }
      }

    } else {
      // assume that the robot is passing and adjust later as needed

      if (OISelector.getOperatorInterface().getPassToggle().getAsBoolean()) {
        targetLandingPosition = Field2d.getInstance().getNearestPassingZone().getTranslation();
        shooterSetpoints =
            getIdealStaticSetpoints(
                targetLandingPosition, passDistanceToVelocityMap, passDistanceToHoodMap, false);

        this.currentMode = ShooterMode.MANUAL_PASS;

        // update the setpoints based on the robots velocity for shoot on the move if toggle is
        // enabled
        if (OISelector.getOperatorInterface().getPassOnTheMoveToggle().getAsBoolean()) {
          shooterSetpoints = calculateShootOnTheMove(shooterSetpoints);
          this.currentMode = ShooterMode.PASS_OTM;
        }

        // check if the robot is in the high pass zone and override the hood and flywheel setpoints
        // to be the high pass setpoints
        if (Field2d.getInstance().inOpponentAllianceHighPassZone()) {
          shooterSetpoints.flywheelVelocityRPS = FLYWHEEL_PASS_OVER_NET_VELOCITY_RPS;
          shooterSetpoints.hoodAngleRot = HOOD_LOWER_ANGLE_LIMIT_ROT;
        }
        // check if the robot is in the no pass zone and switch to collect and hold mode if so to
        // prevent shooting
        else if (Field2d.getInstance().inNoPassZone()) {
          this.currentMode = ShooterMode.COLLECT_AND_HOLD;
        }

      } else {
        targetLandingPosition = Field2d.getInstance().getHubCenter();
        shooterSetpoints =
            getIdealStaticSetpoints(
                targetLandingPosition, hubDistanceToVelocityMap, hubDistanceToHoodMap, true);

        if (OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
          shooterSetpoints = calculateShootOnTheMove(shooterSetpoints);
        }

        this.currentMode = ShooterMode.COLLECT_AND_HOLD;
      }
    }

    //
    // OVERRIDES
    //

    // if the robot is in the collect and hold state, check if the lock turret for bank shot
    // override is on; if so, override the turret setpoint to be the bank shot turret angle to
    // prepare for the money shot
    if (this.currentMode == ShooterMode.COLLECT_AND_HOLD
        && OISelector.getOperatorInterface().getLockTurretForBankToggle().getAsBoolean()) {
      shooterSetpoints.turretAngleRot = LOCK_SHOT_TURRET_ANGLE_ROT;
    }

    // if the lock shooter manual override is on, override the setpoints to be the bank shot
    // setpoints
    if (OISelector.getOperatorInterface().getLockShooterToggle().getAsBoolean()) {
      shooterSetpoints.flywheelVelocityRPS = LOCK_SHOT_FLYWHEEL_RPS;
      shooterSetpoints.hoodAngleRot = LOCK_SHOT_HOOD_ANGLE_ROT;
      shooterSetpoints.turretAngleRot = LOCK_SHOT_TURRET_ANGLE_ROT;
      this.currentMode = ShooterMode.SHOOTER_LOCKED;
    }

    // if the turret appears to be stuck, rotate in the opposite direction to try to get it unstuck
    if (turretOutsideSetpointTimer.hasElapsed(TURRET_OUTSIDE_SETPOINT_THRESHOLD)) {
      // restart the unjamming timer if it isn't already running
      if (!turretUnJammingTimer.isRunning()) {
        turretUnJammingTimer.restart();
      } else if (turretUnJammingTimer.hasElapsed(TURRET_UNJAM_DURATION)) {
        // if we've been trying to unjam for long enough, reset the turret setpoint to the reference
        // position in hopes that the jam has been cleared
        turretUnJammingTimer.stop();
        turretUnJammingTimer.reset();
      } else {
        // rotate away from the setpoint to try to get it unstuck; the direction to rotate is
        // determined by the sign of the turret error
        if ((shooter.getTurretPositionRot() - shooter.getTurretReferencePositionRot()) > 0.0) {
          shooterSetpoints.turretAngleRot =
              shooterSetpoints.turretAngleRot + TURRET_STUCK_ROTATION_ADJUSTMENT_ROT;
        } else {
          shooterSetpoints.turretAngleRot =
              shooterSetpoints.turretAngleRot - TURRET_STUCK_ROTATION_ADJUSTMENT_ROT;
        }

        // ensure that we don't wrap around, which would result in the turret turning the opposite
        // direction and not unjamming
        if (shooterSetpoints.turretAngleRot < TURRET_LOWER_ANGLE_LIMIT_ROT) {
          shooterSetpoints.turretAngleRot = TURRET_LOWER_ANGLE_LIMIT_ROT;
        } else if (shooterSetpoints.turretAngleRot > TURRET_UPPER_ANGLE_LIMIT_ROT) {
          shooterSetpoints.turretAngleRot = TURRET_UPPER_ANGLE_LIMIT_ROT;
        }
      }
    }

    if (OISelector.getOperatorInterface().getSlowShooterForPitTest().getAsBoolean()) {
      shooterSetpoints.flywheelVelocityRPS = PIT_TEST_FLYWHEEL_RPS;
      shooterSetpoints.hoodAngleRot = HOOD_MAX_PASSING_ANGLE_ROT;
    }

    // do not run the flywheels if we are racing to the middle in auto
    if (DriverStation.isAutonomousEnabled()
        && AutonomousCommandsFactory.getInstance().getCustomMatchTime() < 3.0) {
      shooterSetpoints.flywheelVelocityRPS = 0.0;
    }

    // finally, override the hood position if the robot is in a trench zone to ensure that the
    // shooter doesn't get decapitated
    if (Field2d.getInstance().inTrenchZone()
        && shooterSetpoints.hoodAngleRot > HOOD_NEAR_TRENCH_ANGLE_LIMIT_ROT) {
      this.currentMode = ShooterMode.NEAR_TRENCH;
      shooterSetpoints.hoodAngleRot = HOOD_NEAR_TRENCH_ANGLE_LIMIT_ROT;
      LEDs.getInstance().requestState(LEDs.States.IN_TRENCH_ZONE);
    }

    shooter.setFlywheelVelocity(shooterSetpoints.flywheelVelocityRPS);
    shooter.setHoodPosition(shooterSetpoints.hoodAngleRot);
    shooter.setTurretPosition(shooterSetpoints.turretAngleRot);

    Logger.recordOutput(
        "ShooterModes/targetLandingPose", new Pose2d(targetLandingPosition, new Rotation2d()));
    Logger.recordOutput(
        "ShooterModes/FlywheelVelocitySetpointRPS", shooterSetpoints.flywheelVelocityRPS);
    Logger.recordOutput("ShooterModes/HoodAngleSetpointDegrees", shooterSetpoints.hoodAngleRot);
    Logger.recordOutput("ShooterModes/TurretAngleSetpointDegrees", shooterSetpoints.turretAngleRot);
  }

  private ShooterSetpoints calculateShootOnTheMove(ShooterSetpoints staticSetpoints) {
    double v =
        staticSetpoints.flywheelVelocityRPS
            * FLYWHEEL_VELOCITY_SCALE_FACTOR
            * Math.PI
            * Units.inchesToMeters(3);
    double theta = Units.rotationsToRadians(staticSetpoints.hoodAngleRot);

    Pose2d robotPose = RobotOdometry.getInstance().getEstimatedPose();
    double phi =
        new Rotation2d(staticSetpoints.turretAngleRot).plus(robotPose.getRotation()).getRadians();

    ChassisSpeeds fieldRelativeSpeeds = getShooterFieldRelativeVelocity();
    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    double newFlywheelVelocityRPS =
        (Math.sqrt(
                Math.pow(v * Math.sin(theta) * Math.cos(phi) - robotVx, 2)
                    + Math.pow(v * Math.sin(theta) * Math.sin(phi) - robotVy, 2)
                    + Math.pow(v * Math.cos(theta), 2)))
            / (Math.PI * FLYWHEEL_VELOCITY_SCALE_FACTOR * Units.inchesToMeters(3));

    double newHoodAngle =
        (180.0 / Math.PI)
            * Math.atan2(
                Math.sqrt(
                    Math.pow(v * Math.sin(theta) * Math.cos(phi) - robotVx, 2)
                        + Math.pow(v * Math.sin(theta) * Math.sin(phi) - robotVy, 2)),
                v * Math.cos(theta));

    double newTurretAngle =
        new Rotation2d(
                Math.atan2(
                    (v * Math.sin(theta) * Math.sin(phi) - robotVy),
                    (v * Math.sin(theta) * Math.cos(phi) - robotVx)))
            .minus(robotPose.getRotation())
            .getDegrees();

    return new ShooterSetpoints(
        newFlywheelVelocityRPS,
        Units.degreesToRotations(newHoodAngle),
        Units.degreesToRotations(newTurretAngle));
  }

  // increases shot velocities by 1%
  public void incrementShotVelocity() {
    this.shotVelocityMultiplier += 0.01;
  }

  // decreases shot velocities by 1%
  public void decrementShotVelocity() {
    this.shotVelocityMultiplier -= 0.01;
  }

  // increases turret angle by 1 deg
  public void moveTurretOneDegreeLeft() {
    this.turretAngleAdjustmentDeg += 1.0;
  }

  // decreases turret angle by 1 deg
  public void moveTurretOneDegreeRight() {
    this.turretAngleAdjustmentDeg -= 1.0;
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

  private ShooterSetpoints getIdealStaticSetpoints(
      Translation2d targetLandingPosition,
      InterpolatingDoubleTreeMap velocityMap,
      InterpolatingDoubleTreeMap hoodMap,
      boolean isShootingHub) {
    // find our distances to target in x, y and theta

    // transform robot pose by calculated robot to shooter transform
    Pose2d robotPose =
        RobotOdometry.getInstance().getEstimatedPose().transformBy(ROBOT_TO_TURRET_TRANSFORM);

    double deltaX = targetLandingPosition.getX() - robotPose.getX();
    double deltaY = targetLandingPosition.getY() - robotPose.getY();
    double distance = Math.hypot(deltaY, deltaX);
    Logger.recordOutput("/ShooterModes/DistanceToHub", distance);

    double idealShotVelocityRPS = velocityMap.get(distance);

    double fieldRelativeTurretAngleRad = Math.atan2(deltaY, deltaX);
    Rotation2d robotRelativeTurretAngleRadians =
        new Rotation2d(fieldRelativeTurretAngleRad).minus(robotPose.getRotation());
    double robotRelativeTurretAngleRot = robotRelativeTurretAngleRadians.getRotations();

    double idealHoodAngleRot = Units.degreesToRotations(hoodMap.get(distance));

    // if we are shooting into the hub, apply the shot velocity multiplier (don't apply for passes)
    if (isShootingHub) {
      idealShotVelocityRPS *= this.shotVelocityMultiplier;
      robotRelativeTurretAngleRot =
          robotRelativeTurretAngleRot + Units.degreesToRotations(this.turretAngleAdjustmentDeg);
    }

    return new ShooterSetpoints(
        idealShotVelocityRPS, idealHoodAngleRot, robotRelativeTurretAngleRot);
  }
}
