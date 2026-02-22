package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

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
      OperatorInterface oi, SwerveDrivetrain swerveDrivetrain, Intake intake, Hopper hopper, Vision vision) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(swerveDrivetrain, intake, hopper, vision, oi));

    oi.getDriveToPoseButton().onTrue(getDriveToPoseCommand(swerveDrivetrain, oi));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    registerSysIdCommands(oi);
  }

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain, Vision vision) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(differentialDrivetrain, vision, oi));

    registerSysIdCommands(oi);
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
      SwerveDrivetrain swerveDrivetrain, Hopper hopper, Vision vision, OperatorInterface oi) {
    return Commands.parallel(
            new TeleopSwerve(swerveDrivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
            Commands.runOnce(intake::stopRoller),
            Commands.runOnce(hopper::stopKicker),
            Commands.runOnce(hopper::stopSpindexer))
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

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  private static Pose2d getTargetPose() {
    return new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(90.0));
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
