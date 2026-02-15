package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shooter.Shooter;
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
      Climber climber,
      Shooter shooter) {

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(swerveDrivetrain, vision, arm, elevator, shooter, oi));

    oi.getDriveToPoseButton().onTrue(getDriveToPoseCommand(swerveDrivetrain, elevator, oi));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    oi.getDriveToLeftClimbButton()
        .and(() -> inClimbingTolerance(swerveDrivetrain.getPose()))
        .onTrue(getDriveToLeftClimbCommand(swerveDrivetrain, climber, oi));

    oi.getDriveToRightClimbButton()
        .and(() -> inClimbingTolerance(swerveDrivetrain.getPose()))
        .onTrue(getDriveToRightClimbCommand(swerveDrivetrain, climber, oi));

    registerSysIdCommands(oi);
  }

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain, Vision vision, Arm arm) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(differentialDrivetrain, vision, arm, oi));

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

  private static Command getDriveToLeftClimbCommand(
      SwerveDrivetrain swerveDrivetrain, Climber climber, OperatorInterface oi) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.parallel(
            new DriveToPose(
                swerveDrivetrain,
                CrossSubsystemsCommandsFactory::getLeftClimbPose,
                xController,
                yController,
                thetaController,
                new Transform2d(0.05, 0.05, Rotation2d.fromDegrees(2.0)),
                true,
                (atPose) ->
                    LEDs.getInstance()
                        .requestState(
                            atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
                CrossSubsystemsCommandsFactory::updatePIDConstants,
                5.0),
            Commands.runOnce(() -> climber.setClimberAngle(ClimberConstants.CLIMB_READY_ANGLE)))
        .withName("drive to left climb");
  }

  private static Command getDriveToRightClimbCommand(
      SwerveDrivetrain swerveDrivetrain, Climber climber, OperatorInterface oi) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.parallel(
            new DriveToPose(
                swerveDrivetrain,
                CrossSubsystemsCommandsFactory::getRightClimbPose,
                xController,
                yController,
                thetaController,
                new Transform2d(0.05, 0.05, Rotation2d.fromDegrees(2.0)),
                true,
                (atPose) ->
                    LEDs.getInstance()
                        .requestState(
                            atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
                CrossSubsystemsCommandsFactory::updatePIDConstants,
                5.0),
            Commands.runOnce(() -> climber.setClimberAngle(ClimberConstants.CLIMB_READY_ANGLE)))
        .withName("drive to right climb");
  }

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  private static Pose2d getTargetPose() {
    return new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(90.0));
  }

  private static Pose2d getLeftClimbPose() {
    Translation2d targetPosition;

    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      targetPosition = FieldConstants.Tower.leftUpright;
    } else {
      targetPosition = FieldConstants.Tower.oppLeftUpright;
    }

    double yOffset =
        RobotConfig.getInstance().getRobotWidthWithBumpers().in(Meters) / 2; // FIXME: adjust offset

    double xOffset =
        RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters)
            / 2; // FIXME: adjust offset

    return new Pose2d(
        targetPosition.getX() + xOffset,
        targetPosition.getY() + yOffset,
        Rotation2d.fromDegrees(-90));
  }

  private static Pose2d getRightClimbPose() {
    Translation2d targetPosition;

    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      targetPosition = FieldConstants.Tower.rightUpright;
    } else {
      targetPosition = FieldConstants.Tower.oppRightUpright;
    }

    double yOffset =
        RobotConfig.getInstance().getRobotWidthWithBumpers().in(Meters) / 2; // FIXME: adjust offset

    double xOffset =
        RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters)
            / 2; // FIXME: adjust offset

    return new Pose2d(
        targetPosition.getX() + xOffset,
        targetPosition.getY() - yOffset,
        Rotation2d.fromDegrees(-90));
  }

  private static boolean inClimbingTolerance(Pose2d currentPose) {
    Translation2d towerCenter;

    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      towerCenter = FieldConstants.Tower.centerPoint;
    } else {
      towerCenter = FieldConstants.Tower.oppCenterPoint;
    }

    double distanceToTower = currentPose.getTranslation().getDistance(towerCenter);
    double maxClimbDistance = Units.inchesToMeters(100); // FIXME: adjust this tolerance

    return distanceToTower < maxClimbDistance;
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
