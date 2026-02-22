package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.shooter.Shooter;

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

  private static final double DRIVE_TO_BANK_X_TOLERANCE_METERS = 0.05;
  private static final double DRIVE_TO_BANK_Y_TOLERANCE_METERS =
      0.25; // high robot-relative y tolerance as it doesn't really matter where on the wall we are
  private static final double DRIVE_TO_BANK_THETA_TOLERANCE_DEGREES = 5.0;

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
      OperatorInterface oi, SwerveDrivetrain swerveDrivetrain, Vision vision, Shooter shooter
      /*, Hopper hopper */ ) {

    configureCrossSubsystemsTriggers(oi, swerveDrivetrain, shooter);

    oi.getInterruptAll().onTrue(getInterruptAllCommand(swerveDrivetrain, vision, shooter, oi));

    oi.getScoreFromBankButton().onTrue(getScoreSafeShotCommand(swerveDrivetrain, /*, hopper*/ oi));

    oi.getManualShootButton()
        // .and(shooterModes::manualShootEnabled)
        .whileTrue(getUnloadShooterCommand(swerveDrivetrain));

    oi.getManualShootButton().onFalse(Commands.none()); // stop hopper

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    registerSysIdCommands(oi);
  }

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain, Vision vision) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(differentialDrivetrain, vision, oi));

    registerSysIdCommands(oi);
  }

  // this will get called if we are in CAN_SHOOT mode AND the aim button is pressed
  public static Command getScoreSafeShotCommand(
      SwerveDrivetrain drivetrain /*, Hopper hopper */, OperatorInterface oi) {
    // ShooterModes shooterModes) {

    return // Commands.either(
    Commands.sequence(
        getDriveToBankCommand(drivetrain), getUnloadShooterCommand(drivetrain /*, hopper*/));
    // Commands.none());
    // () -> shooterModes.manualShootEnabled());
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
              RobotConfig.getInstance()
                  .getRobotMaxVelocity()
                  .times(OISelector.getOperatorInterface().getTranslateX()),
              RobotConfig.getInstance()
                  .getRobotMaxVelocity()
                  .times(OISelector.getOperatorInterface().getTranslateY()),
              targetRotation,
              false);
        });
  }

  // this is called in the sequence of getScoreSafeShot or while we hold right trigger 1 in
  // CAN_SHOOT / non SHOOT_OTM
  public static Command getUnloadShooterCommand(SwerveDrivetrain drivetrain /*, Hopper hopper */) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::holdXstance), Commands.none()); // FIXME: run hopper
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
      SwerveDrivetrain swerveDrivetrain, Vision vision, Shooter shooter, OperatorInterface oi) {
    return Commands.parallel(
            new TeleopSwerve(swerveDrivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
            Commands.runOnce(shooter::stopHood, shooter))
        .withName("interrupt all");
  }

  private static Command getInterruptAllCommand(
      DifferentialDrivetrain differentialDrivetrain, Vision vision, OperatorInterface oi) {
    return Commands.parallel(
            new ArcadeDrive(differentialDrivetrain, oi::getTranslateX, oi::getRotate))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseCommand(
      SwerveDrivetrain swerveDrivetrain, OperatorInterface oi) {
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
            () -> Field2d.getInstance().getNearestBank(),
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

  private static Pose2d getTargetPassingZonePose() {
    return Field2d.getInstance().getNearestPassingZone();
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

  private static void configureCrossSubsystemsTriggers(
      OperatorInterface oi,
      SwerveDrivetrain swerveDrivetrain,
      // ShooterModes shooterModes,
      Shooter shooter) {

    Trigger unloadHopperOnTheMoveTrigger =
        new Trigger(
            () ->
                Field2d.getInstance()
                    .inAllianceZone()); //  && shooterModes.isShootOnTheMoveEnabled());

    // FIXME: when running in sim, used a run once to set a constant velocity.
    // we can do this worst case, or try using a whileTrue + runOnce (so that we constantly update
    // speed),
    // followed by an onFalse to stop the hopper when we stop moving.
    unloadHopperOnTheMoveTrigger.onTrue(Commands.none());
    unloadHopperOnTheMoveTrigger.onFalse(Commands.none());

    Trigger unloadHopperForPassingTrigger =
        new Trigger(
            () -> !Field2d.getInstance().inAllianceZone()); // && shooterModes.isPassEnabled());

    // FIXME: same note as above
    unloadHopperForPassingTrigger.onTrue(Commands.none());
    unloadHopperForPassingTrigger.onFalse(Commands.none());
  }
}
