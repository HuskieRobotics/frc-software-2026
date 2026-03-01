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
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterModes;
import java.util.Optional;

public class CrossSubsystemsCommandsFactory {

  private static final LoggedTunableNumber driveXKp =
      new LoggedTunableNumber(
          "DriveToBank/DriveXKp", RobotConfig.getInstance().getDriveToPoseDriveXKP());
  private static final LoggedTunableNumber driveYKp =
      new LoggedTunableNumber(
          "DriveToBank/DriveYKp", RobotConfig.getInstance().getDriveToPoseDriveYKP());
  private static final LoggedTunableNumber driveXKd =
      new LoggedTunableNumber(
          "DriveToBank/DriveXKd", RobotConfig.getInstance().getDriveToPoseDriveXKD());
  private static final LoggedTunableNumber driveYKd =
      new LoggedTunableNumber(
          "DriveToBank/DriveYKd", RobotConfig.getInstance().getDriveToPoseDriveYKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToBank/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToBank/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToBank/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToBank/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private static final LoggedTunableNumber driveToPoseMaxVelocity =
      new LoggedTunableNumber(
          "DriveToBank/DriveToPoseMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity().in(MetersPerSecond));
  private static final LoggedTunableNumber driveToPoseMaxAcceleration =
      new LoggedTunableNumber(
          "DriveToBank/DriveToPoseMaxAcceleration",
          RobotConfig.getInstance()
              .getDriveToPoseDriveMaxAcceleration()
              .in(MetersPerSecondPerSecond));

  private static final double DRIVE_TO_BANK_X_TOLERANCE_METERS = 0.05;
  private static final double DRIVE_TO_BANK_Y_TOLERANCE_METERS =
      0.25; // high robot-relative y tolerance as it doesn't really matter where on the wall we are
  private static final double DRIVE_TO_BANK_THETA_TOLERANCE_DEGREES = 5.0;

  public static final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveXKp.get(),
          driveKi.get(),
          driveXKd.get(),
          new TrapezoidProfile.Constraints(
              driveToPoseMaxVelocity.get(), driveToPoseMaxAcceleration.get()));
  public static final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveYKp.get(),
          driveKi.get(),
          driveYKd.get(),
          new TrapezoidProfile.Constraints(
              driveToPoseMaxVelocity.get(), driveToPoseMaxAcceleration.get()));
  public static final ProfiledPIDController thetaController =
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
      Intake intake,
      Hopper hopper,
      Shooter shooter,
      ShooterModes shooterModes) {

    configureCrossSubsystemsTriggers(shooterModes, shooter, hopper);

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(swerveDrivetrain, intake, hopper, shooter, oi));

    new Trigger(shooterModes::isManualShootEnabled)
        .or(shooterModes::isLockedShooterEnabled)
        .and(oi.getScoreFromBankButton())
        .onTrue(
            getScoreSafeShotCommand(oi, swerveDrivetrain, shooter, hopper, intake, shooterModes));

    oi.getManualShootButton()
        .and(new Trigger(shooterModes::isManualShootEnabled).or(shooterModes::isPassEnabled))
        .onTrue(
            getStopAndShootCommand(oi, swerveDrivetrain, shooter, hopper, intake, shooterModes));

    oi.getManualShootButton()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(hopper::stop, hopper),
                Commands.runOnce(intake::deployIntake, intake)));

    oi.getSnakeDriveButton().toggleOnTrue(getSnakeDriveCommand(oi, swerveDrivetrain));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    registerSysIdCommands(oi);
  }

  // this will get called if we are in CAN_SHOOT mode AND the aim button is pressed
  public static Command getScoreSafeShotCommand(
      OperatorInterface oi,
      SwerveDrivetrain drivetrain,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShooterModes shooterModes) {
    return Commands.sequence(
            getDriveToBankCommand(drivetrain),
            getStopAndShootCommand(oi, drivetrain, shooter, hopper, intake, shooterModes))
        .withName("score safe shot");
  }

  public static Command getSnakeDriveCommand(OperatorInterface oi, SwerveDrivetrain drivetrain) {
    return new TeleopSwerve(
            drivetrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            () -> Optional.of(new Rotation2d(oi.getTranslateX(), oi.getTranslateY())))
        .unless(
            () -> !OISelector.getOperatorInterface().getAutoSnapsEnabledTrigger().getAsBoolean())
        .withName("Snake Drive Command");
  }

  // this is called in the sequence of getScoreSafeShot or while we hold right trigger 1 in
  // CAN_SHOOT / non SHOOT_OTM
  public static Command getStopAndShootCommand(
      OperatorInterface oi,
      SwerveDrivetrain drivetrain,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShooterModes shooterModes) {
    return Commands.sequence(
            Commands.runOnce(drivetrain::holdXstance, drivetrain),
            Commands.parallel(
                    Commands.run(
                        () -> hopper.feedFuelIntoShooter(shooter.getFlywheelLeadVelocity()),
                        hopper),
                    Commands.run(intake::jostleFuel, intake),
                    Commands.either(
                        Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                        Commands.run(() -> LEDs.getInstance().requestState(States.SHOOTING)),
                        shooterModes::isPassEnabled))
                .until(
                    () ->
                        (Math.abs(oi.getTranslateX()) > 0.1 || Math.abs(oi.getTranslateY()) > 0.1)),
            Commands.runOnce(intake::deployIntake, intake),
            Commands.runOnce(hopper::stop, hopper))
        .withName("stop and shoot or pass");
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
      Intake intake,
      Hopper hopper,
      Shooter shooter,
      OperatorInterface oi) {
    return Commands.parallel(
            SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(oi, swerveDrivetrain),
            Commands.runOnce(intake::stopRoller, intake),
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(shooter::stopHood, shooter))
        .withName("interrupt all");
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
    return SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(oi, drivetrain)
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

  public static void updatePIDConstants(Transform2d poseDifference) {
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
      ShooterModes shooterModes, Shooter shooter, Hopper hopper) {

    Trigger unloadHopperOnTheMoveTrigger = new Trigger(shooterModes::isShootOnTheMoveEnabled);
    unloadHopperOnTheMoveTrigger.whileTrue(
        Commands.parallel(
                Commands.runOnce(
                    () -> hopper.feedFuelIntoShooter(shooter.getFlywheelLeadVelocity()), hopper),
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.SHOOTING)))
            .withName("feed fuel (shoot)"));
    unloadHopperOnTheMoveTrigger.onFalse(Commands.runOnce(hopper::stop, hopper));

    // when we enable shoot or pass on the move, we will need to add logic to pause the hopper when
    // the turret is flipping to prevent errant shots
    // Trigger turretFlippingSides = new Trigger(shooter::isTurretNearSetPoint);
    // turretFlippingSides.onFalse(
    //     Commands.runOnce(hopper::stop, hopper).withName("pause hopper; turret flipping"));
  }
}
