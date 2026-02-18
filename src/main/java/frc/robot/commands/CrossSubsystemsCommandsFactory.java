package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterModes;
import java.util.List;

public class CrossSubsystemsCommandsFactory {

  private static final LoggedTunableNumber driveXKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveXKp", RobotConfig.getInstance().getDriveToPoseDriveXKP());
  private static final LoggedTunableNumber driveYKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveYKp", RobotConfig.getInstance().getDriveToPoseDriveYKP());
  private static final LoggedTunableNumber driveXKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveXKd", RobotConfig.getInstance().getDriveToPoseDriveXKD());
  private static final LoggedTunableNumber driveYKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveYKd", RobotConfig.getInstance().getDriveToPoseDriveYKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToPoseExample/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private static final double WALL_SNAP_TOLERANCE_METERS = Units.inchesToMeters(30);

  private static final double DRIVE_TO_BANK_X_TOLERANCE_METERS = 0.05;
  private static final double DRIVE_TO_BANK_Y_TOLERANCE_METERS =
      0.25; // high robot-relative y tolerance as it doesn't really matter where on the wall we are
  private static final double DRIVE_TO_BANK_THETA_TOLERANCE_DEGREES = 5.0;

  private static final double JOYSTICK_POWER = 2.0;

  private static ProfiledPIDController xController =
      new ProfiledPIDController(
          driveXKp.get(),
          driveKi.get(),
          driveXKd.get(),
          new TrapezoidProfile.Constraints(
              RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity().in(MetersPerSecond),
              RobotConfig.getInstance()
                  .getDriveToPoseDriveMaxAcceleration()
                  .in(MetersPerSecondPerSecond)));
  private static ProfiledPIDController yController =
      new ProfiledPIDController(
          driveYKp.get(),
          driveKi.get(),
          driveYKd.get(),
          new TrapezoidProfile.Constraints(
              RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity().in(MetersPerSecond),
              RobotConfig.getInstance()
                  .getDriveToPoseDriveMaxAcceleration()
                  .in(MetersPerSecondPerSecond)));
  private static ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(
              RobotConfig.getInstance().getDriveToPoseTurnMaxVelocity().in(RadiansPerSecond),
              RobotConfig.getInstance()
                  .getDriveToPoseTurnMaxAcceleration()
                  .in(RadiansPerSecondPerSecond)));

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi,
      SwerveDrivetrain swerveDrivetrain,
      Vision vision,
      Arm arm,
      Elevator elevator,
      Shooter shooter,
      ShooterModes shooterModes
      /*, Hopper hopper */ ) {

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(swerveDrivetrain, vision, arm, elevator, shooter, oi));

    oi.getScoreFromBankButton()
        .onTrue(getScoreSafeShotCommand(swerveDrivetrain /*, hopper*/, oi, shooterModes));

    oi.getSnakeDriveButton().toggleOnTrue(getSnakeDriveCommand(swerveDrivetrain));

    Trigger manualShootTrigger =
        oi.getManualShootButton()
            .and(shooterModes::manualShootEnabled)
            .whileTrue(getUnloadShooterCommand(swerveDrivetrain));

    new Trigger(
            () -> {
              double currentYPose = swerveDrivetrain.getPose().getY();

              // see which of the two walls we are closer to
              boolean closerToLeft = currentYPose < WALL_SNAP_TOLERANCE_METERS;
              boolean closerToRight =
                  currentYPose > (FieldConstants.fieldWidth - WALL_SNAP_TOLERANCE_METERS);

              return closerToLeft || closerToRight;
            })
        .whileTrue(getSnapToWallsCommand(swerveDrivetrain));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    registerSysIdCommands(oi);
  }

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain, Vision vision, Arm arm) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(differentialDrivetrain, vision, arm, oi));

    registerSysIdCommands(oi);
  }

  // this will get called if we are in CAN_SHOOT mode AND the aim button is pressed
  public static Command getScoreSafeShotCommand(
      SwerveDrivetrain drivetrain /*, Hopper hopper */,
      OperatorInterface oi,
      ShooterModes shooterModes) {

    // check if we are in CAN_SHOOT mode: either grab mode directly (figure out how) or check OI !=
    // shoot_otm && in AZ

    return Commands.either(
        Commands.sequence(
            getDriveToBankCommand(drivetrain), getUnloadShooterCommand(drivetrain /*, hopper*/)),
        Commands.none(),
        /* change this check to be the getter method in ShooterModes for CAN_SHOOT mode (which will have this condition)  */
        () -> shooterModes.manualShootEnabled());
  }

  private static double getModifiedXTranslation() {
    return TeleopSwerve.modifyAxis(
            OISelector.getOperatorInterface().getTranslateX(), JOYSTICK_POWER)
        * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);
  }

  private static double getModifiedYTranslation() {
    return TeleopSwerve.modifyAxis(
            OISelector.getOperatorInterface().getTranslateY(), JOYSTICK_POWER)
        * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);
  }

  // this will rotate our robot into a diamond shape while we are near the bump zone
  public static Command getRotateWhileNearBumpCommand(SwerveDrivetrain drivetrain) {

    return Commands.run(
        () -> {
          double currentRotationPose = drivetrain.getPose().getRotation().getDegrees();

          double nearest45DegreeAngle =
              (Math.round(((currentRotationPose - 45) / 90)) * 90)
                  + 45; // this should grab the nearest diamond angle (45+90x)

          Rotation2d targetRotation = Rotation2d.fromDegrees(nearest45DegreeAngle);

          drivetrain.driveFacingAngle(
              MetersPerSecond.of(getModifiedXTranslation()),
              MetersPerSecond.of(getModifiedYTranslation()),
              targetRotation,
              false);
        });
  }

  public static Command getSnapToWallsCommand(SwerveDrivetrain drivetrain) {
    return Commands.run(
            () -> {
              Rotation2d currentRotation = drivetrain.getPose().getRotation();
              double currentDegrees = currentRotation.getDegrees();

              double nearest90DegreeAngle =
                  Math.round(currentDegrees / 90.0) * 90.0; // nearest multiple of 90 degrees

              Rotation2d targetRotation = Rotation2d.fromDegrees(nearest90DegreeAngle);

              drivetrain.driveFacingAngle(
                  MetersPerSecond.of(getModifiedXTranslation()),
                  MetersPerSecond.of(getModifiedYTranslation()),
                  targetRotation,
                  false);
            })
        .withName("Auto Snap To 90 Degree While Near Wall Command");
  }

  public static Command getSnakeDriveCommand(SwerveDrivetrain drivetrain) {
    return Commands.run(
            () -> {
              double xVelocity = getModifiedXTranslation();
              double yVelocity = getModifiedYTranslation();
              Rotation2d targetRotation = new Rotation2d(xVelocity, yVelocity);

              drivetrain.driveFacingAngle(
                  MetersPerSecond.of(xVelocity),
                  MetersPerSecond.of(yVelocity),
                  targetRotation,
                  false);
            })
        .withName("Snake Drive Command");
  }

  // this is called in the sequence of getScoreSafeShot or while we hold right trigger 1 in
  // CAN_SHOOT / non SHOOT_OTM
  public static Command getUnloadShooterCommand(SwerveDrivetrain drivetrain /*, Hopper hopper */) {

    return Commands.sequence(Commands.runOnce(drivetrain::holdXstance));
    // add hopper kick method in parallel
  }

  private static void registerSysIdCommands(OperatorInterface oi) {
    oi.getSysIdDynamicForward().whileTrue(SysIdRoutineChooser.getInstance().getDynamicForward());
    oi.getSysIdDynamicReverse().whileTrue(SysIdRoutineChooser.getInstance().getDynamicReverse());
    oi.getSysIdQuasistaticForward()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticForward());
    oi.getSysIdQuasistaticReverse()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticReverse());
  }

  private static Command getInterruptAllCommand(
      SwerveDrivetrain swerveDrivetrain,
      Vision vision,
      Arm arm,
      Elevator elevator,
      Shooter shooter,
      OperatorInterface oi) {
    return Commands.parallel(
            new TeleopSwerve(swerveDrivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(() -> arm.setAngle(Degrees.of(0.0)), arm),
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.Positions.BOTTOM), elevator),
            Commands.runOnce(() -> shooter.setIdleVelocity(), shooter))
        .withName("interrupt all");
  }

  private static Command getInterruptAllCommand(
      DifferentialDrivetrain differentialDrivetrain, Vision vision, Arm arm, OperatorInterface oi) {
    return Commands.parallel(
            new ArcadeDrive(differentialDrivetrain, oi::getTranslateX, oi::getRotate),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(() -> arm.setAngle(Degrees.of(0.0)), arm))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseCommand(
      SwerveDrivetrain swerveDrivetrain, Elevator elevator, OperatorInterface oi) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return new DriveToPose(
            swerveDrivetrain,
            CrossSubsystemsCommandsFactory::getTargetPose,
            xController,
            yController,
            thetaController,
            new Transform2d(0.10, 0.05, Rotation2d.fromDegrees(5.0)),
            true,
            (atPose) ->
                LEDs.getInstance()
                    .requestState(atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
            CrossSubsystemsCommandsFactory::updatePIDConstants,
            5.0)
        .withName("drive to pose");
  }

  private static Command getDriveToBankCommand(SwerveDrivetrain drivetrain) {
    return new DriveToBank(
            drivetrain,
            CrossSubsystemsCommandsFactory::getTargetBankPose,
            xController,
            yController,
            thetaController,
            new Transform2d(
                DRIVE_TO_BANK_X_TOLERANCE_METERS,
                DRIVE_TO_BANK_Y_TOLERANCE_METERS,
                Rotation2d.fromDegrees(DRIVE_TO_BANK_THETA_TOLERANCE_DEGREES)),
            true,
            (atPose) ->
                LEDs.getInstance()
                    .requestState(
                        atPose
                            ? LEDs.States.AT_POSE
                            : LEDs.States
                                .AUTO_DRIVING_TO_POSE), /* will do other in parallel, probably not here though */
            CrossSubsystemsCommandsFactory::updatePIDConstants, /* need to see this */
            3.0)
        .withName("drive to bank");
  }

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  private static Pose2d getTargetPose() {
    return new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(90.0));
  }

  private static Pose2d getTargetBankPose() {
    // for testing
    // return new Pose2d(FieldConstants.LinesVertical.center, FieldConstants.LinesHorizontal.center,
    // Rotation2d.fromDegrees(90));
    return Field2d.getInstance().getNearestBank();
  }

  private static void updatePIDConstants(Transform2d poseDifference) {
    // Update from tunable numbers
    LoggedTunableNumber.ifChanged(
        xController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveXKp,
        driveKi,
        driveXKd);

    LoggedTunableNumber.ifChanged(
        yController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveYKp,
        driveKi,
        driveYKd);

    LoggedTunableNumber.ifChanged(
        thetaController.hashCode(),
        pid -> {
          thetaController.setPID(pid[0], pid[1], pid[2]);
        },
        thetaKp,
        thetaKi,
        thetaKd);
  }
}
