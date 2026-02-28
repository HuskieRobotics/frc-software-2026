package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

public class ShooterModes extends SubsystemBase {

  private static final double SHOOT_TIME_OFFSET_SECONDS = 2.0; // offset for ball travel time to hub
  private static final double END_OF_SHIFT_WARNING_SECONDS =
      5.0; // time before the end of the shift to flash the LE#Ds

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

  private final LoggedTunableNumber shooterTestingEnable =
      new LoggedTunableNumber("ShooterModes/Testing/EnableTesting", 0.0);
  private final LoggedTunableNumber testingFlywheelVelocity =
      new LoggedTunableNumber("ShooterModes/Testing/FlywheelVelocityRPS", 0.0);
  private final LoggedTunableNumber testingHoodAngle =
      new LoggedTunableNumber("ShooterModes/Testing/HoodAngleDegrees", HOOD_MIN_ANGLE.in(Degrees));
  private final LoggedTunableNumber testingTurretAngle =
      new LoggedTunableNumber("ShooterModes/Testing/TurretAngleDegrees", 0.0);

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trench zone
    PASS, // passing mode,
    SHOOTER_LOCKED, // set our manual hood, turret, and flywheel values for money shot
    TESTING // testing mode for testing
  }

  private class ShooterSetpoints {
    public AngularVelocity flywheelVelocity;
    public Angle hoodAngle;
    public Angle turretAngle;

    public ShooterSetpoints(AngularVelocity flywheelVelocity, Angle hoodAngle, Angle turretAngle) {
      this.flywheelVelocity = flywheelVelocity;
      this.hoodAngle = hoodAngle;
      this.turretAngle = turretAngle;
    }
  }

  private ShooterMode currentMode = ShooterMode.COLLECT_AND_HOLD;

  public ShooterModes(SwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hubActive = OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean();

    populateMaps();
  }

  @Override
  public void periodic() {
    this.hubActive = isHubActive();

    determineModeAndSetShooter();

    Logger.recordOutput("ShooterModes/CurrentMode", this.currentMode);
    Logger.recordOutput("ShooterModes/HubActive", this.hubActive);
  }

  private void populateMaps() {
    // Velocity Shooting into Hub Map
    hubDistanceToVelocityMap.put(1.8034, 28.0);
    hubDistanceToVelocityMap.put(2.032, 29.0);
    hubDistanceToVelocityMap.put(2.413, 30.0);
    hubDistanceToVelocityMap.put(2.8702, 32.0);
    hubDistanceToVelocityMap.put(2.9972, 33.0);
    hubDistanceToVelocityMap.put(3.2512, 34.0);
    hubDistanceToVelocityMap.put(3.83, 35.0);
    hubDistanceToVelocityMap.put(4.4196, 38.0);
    hubDistanceToVelocityMap.put(4.72, 39.0);
    hubDistanceToVelocityMap.put(5.38, 41.0);

    // Hood Angle Shooting into Hub Map
    hubDistanceToHoodMap.put(1.8034, 22.0);
    hubDistanceToHoodMap.put(2.032, 22.0);
    hubDistanceToHoodMap.put(2.413, 22.0);
    hubDistanceToHoodMap.put(2.8702, 23.0);
    hubDistanceToHoodMap.put(2.9972, 23.0);
    hubDistanceToHoodMap.put(3.2512, 25.0);
    hubDistanceToHoodMap.put(3.83, 26.0);
    hubDistanceToHoodMap.put(4.4196, 27.0);
    hubDistanceToHoodMap.put(4.72, 27.0);
    hubDistanceToHoodMap.put(5.38, 28.0);

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

  public boolean isPassEnabled() {
    return this.currentMode == ShooterMode.PASS;
  }

  public boolean isManualShootEnabled() {
    return this.currentMode == ShooterMode.MANUAL_SHOOT;
  }

  public boolean isLockedShooterEnabled() {
    return this.currentMode == ShooterMode.SHOOTER_LOCKED;
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub

  // The alliance will be provided as a single character representing the color of the alliance
  // whose goal will go inactive first
  // (i.e. ‘R’ = red, ‘B’ = blue). This alliance’s goal will be active in Shifts 2 and 4.

  public boolean isHubActive() {
    if (!DriverStation.isFMSAttached()) {
      return OISelector.getOperatorInterface().getHubActiveAtHomeToggle().getAsBoolean();
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

      // each shift is 25 seconds long, request the LED state before the end of each shift
      if ((130 - timeRemaining) % 25 < END_OF_SHIFT_WARNING_SECONDS) {
        LEDs.getInstance().requestState(LEDs.States.END_OF_PERIOD);
      }
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
    if (shooterTestingEnable.get() != 0) {
      this.currentMode = ShooterMode.TESTING;
      shooter.setFlywheelVelocity(RotationsPerSecond.of(testingFlywheelVelocity.get()));
      shooter.setHoodPosition(Degrees.of(testingHoodAngle.get()));
      shooter.setTurretPosition(Degrees.of(testingTurretAngle.get()));
      return;
    }

    // determine if the robot will be potentially shooting or passing, which is based on  whether we
    // are in the alliance zone or not

    ShooterSetpoints shooterSetpoints;
    Translation2d targetLandingPosition;

    if (Field2d.getInstance().inAllianceZone()) {
      // assume that if the robot is shooting from rest and adjust later as needed
      targetLandingPosition = Field2d.getInstance().getHubCenter();
      shooterSetpoints = getIdealStaticShotSetpoints(targetLandingPosition);

      // if the hub is not active, put the robot in collect and hold mode to prepare for when the
      // hub becomes active
      if (!this.hubActive) {
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
        shooterSetpoints = getIdealPassSetpoints(targetLandingPosition);
        shooterSetpoints = calculateShootOnTheMove(shooterSetpoints);

        this.currentMode = ShooterMode.PASS;

        // check if the robot is in the high pass zone and override the hood and flywheel setpoints
        // to be the high pass setpoints
        if (Field2d.getInstance().inOpponentAllianceHighPassZone()) {
          shooterSetpoints.flywheelVelocity = FLYWHEEL_PASS_OVER_NET_VELOCITY;
          shooterSetpoints.hoodAngle = HOOD_LOWER_ANGLE_LIMIT;
        }
        // check if the robot is in the no pass zone and switch to collect and hold mode if so to
        // prevent shooting
        else if (Field2d.getInstance().inNoPassZone()) {
          this.currentMode = ShooterMode.COLLECT_AND_HOLD;
        }

      } else {
        targetLandingPosition = Field2d.getInstance().getHubCenter();
        shooterSetpoints = getIdealStaticShotSetpoints(targetLandingPosition);

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
      shooterSetpoints.turretAngle = LOCK_SHOT_TURRET_ANGLE;
    }

    // if the lock shooter manual override is on, override the setpoints to be the bank shot
    // setpoints
    if (OISelector.getOperatorInterface().getLockShooterToggle().getAsBoolean()) {
      shooterSetpoints.flywheelVelocity = LOCK_SHOT_FLYWHEEL_RPS;
      shooterSetpoints.hoodAngle = LOCK_SHOT_HOOD_ANGLE;
      shooterSetpoints.turretAngle = LOCK_SHOT_TURRET_ANGLE;
      this.currentMode = ShooterMode.SHOOTER_LOCKED;
    }

    if (OISelector.getOperatorInterface().getSlowShooterForPitTest().getAsBoolean()) {
      shooterSetpoints.flywheelVelocity = PIT_TEST_FLYWHEEL_RPS;
    }

    // finally, override the hood position if the robot is in a trench zone to ensure that the
    // shooter doesn't get decapitated
    if (Field2d.getInstance().inTrenchZone()) {
      this.currentMode = ShooterMode.NEAR_TRENCH;
      shooterSetpoints.hoodAngle = HOOD_NEAR_TRENCH_ANGLE_LIMIT;
      LEDs.getInstance().requestState(LEDs.States.IN_TRENCH_ZONE);
    }

    shooter.setFlywheelVelocity(shooterSetpoints.flywheelVelocity);
    shooter.setHoodPosition(shooterSetpoints.hoodAngle);
    shooter.setTurretPosition(shooterSetpoints.turretAngle);

    Logger.recordOutput(
        "ShooterModes/targetLandingPose", new Pose2d(targetLandingPosition, new Rotation2d()));
    Logger.recordOutput(
        "ShooterModes/FlywheelVelocitySetpointRPS", shooterSetpoints.flywheelVelocity);
    Logger.recordOutput("ShooterModes/HoodAngleSetpointDegrees", shooterSetpoints.hoodAngle);
    Logger.recordOutput("ShooterModes/TurretAngleSetpointDegrees", shooterSetpoints.turretAngle);
  }

  private ShooterSetpoints calculateShootOnTheMove(ShooterSetpoints staticSetpoints) {
    double v = staticSetpoints.flywheelVelocity.in(RotationsPerSecond);
    Angle theta = staticSetpoints.hoodAngle;
    Angle phi = staticSetpoints.turretAngle;

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

    return new ShooterSetpoints(
        RotationsPerSecond.of(newFlywheelVelocity),
        Degrees.of(newHoodAngle),
        Degrees.of(newTurretAngle));
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

  private ShooterSetpoints getIdealStaticShotSetpoints(Translation2d targetLandingPosition) {
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

    return new ShooterSetpoints(
        RotationsPerSecond.of(idealShotVelocity), idealHoodAngle, robotRelativeTurretAngle);
  }

  private ShooterSetpoints getIdealPassSetpoints(Translation2d targetLandingPosition) {
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

    return new ShooterSetpoints(
        RotationsPerSecond.of(idealShotVelocity), idealHoodAngle, robotRelativeTurretAngle);
  }
}
