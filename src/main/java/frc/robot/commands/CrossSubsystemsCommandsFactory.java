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
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
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

  private static final double PASS_ZONE_TOLERANCE_X = 2.0; // meters
  private static final double PASS_ZONE_TOLERANCE_Y = 1.0; // meters

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
      ShooterModes shooterModes,
      Vision vision) {

    configureCrossSubsystemsTriggers(oi, swerveDrivetrain, shooterModes, shooter, hopper);

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(swerveDrivetrain, intake, hopper, shooter, oi));

    oi.getScoreFromBankButton()
        .and(shooterModes::isManualShootEnabled)
        .onTrue(getScoreSafeShotCommand(swerveDrivetrain, hopper, oi, shooterModes));

    oi.getManualShootButton()
        .and(shooterModes::isManualShootEnabled)
        .whileTrue(getUnloadShooterCommand(swerveDrivetrain, hopper));

    oi.getManualShootButton()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(hopper::stopKicker), Commands.runOnce(hopper::stopSpindexer)));

    oi.getSnakeDriveButton().toggleOnTrue(getSnakeDriveCommand(oi, swerveDrivetrain));
    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    registerSysIdCommands(oi);
  }

  // this will get called if we are in CAN_SHOOT mode AND the aim button is pressed
  public static Command getScoreSafeShotCommand(
      SwerveDrivetrain drivetrain /*, Hopper hopper */, OperatorInterface oi) {

    // check if we are in CAN_SHOOT mode: either grab mode directly (figure out how) or check OI !=
    // shoot_otm && in AZ

    return Commands.sequence(
        getDriveToBankCommand(drivetrain), getUnloadShooterCommand(drivetrain, oi /*, hopper*/));
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
  public static Command getUnloadShooterCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi /*, Hopper hopper */) {

    return Commands.parallel(Commands.run(drivetrain::holdXstance))
        .until(() -> (Math.abs(oi.getTranslateX()) > 0.1 || Math.abs(oi.getTranslateY()) > 0.1));
    // add hopper kick method in parallel
  }

  // this will get called if we are in CAN_SHOOT mode AND the aim button is pressed
  public static Command getScoreSafeShotCommand(
      SwerveDrivetrain drivetrain, Hopper hopper, OperatorInterface oi, ShooterModes shooterModes) {

    return Commands.either(
        Commands.sequence(
            getDriveToBankCommand(drivetrain), getUnloadShooterCommand(drivetrain, hopper)),
        Commands.none(),
        () -> shooterModes.isManualShootEnabled());
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
  public static Command getUnloadShooterCommand(SwerveDrivetrain drivetrain, Hopper hopper) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::holdXstance),
        Commands.sequence(
            Commands.runOnce(hopper::spinFuelIntoKicker),
            Commands.runOnce(hopper::kickFuelIntoShooter)));
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
            Commands.runOnce(intake::stopRoller),
            Commands.runOnce(hopper::stopKicker),
            Commands.runOnce(hopper::stopSpindexer),
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
      OperatorInterface oi,
      SwerveDrivetrain swerveDrivetrain,
      ShooterModes shooterModes,
      Shooter shooter,
      Hopper hopper) {

    Trigger unloadHopperOnTheMoveTrigger =
        new Trigger(
            () -> Field2d.getInstance().inAllianceZone() && shooterModes.isShootOnTheMoveEnabled());

    unloadHopperOnTheMoveTrigger.whileTrue(
        Commands.parallel(
            Commands.runOnce(() -> hopper.setKickerVelocity(shooter.getFlywheelLeadVelocity())),
            Commands.runOnce(hopper::spinFuelIntoKicker)));
    unloadHopperOnTheMoveTrigger.onFalse(
        Commands.parallel(
            Commands.runOnce(hopper::stopKicker), Commands.runOnce(hopper::stopSpindexer)));

    Trigger unloadHopperForPassingTrigger =
        new Trigger(() -> !Field2d.getInstance().inAllianceZone() && shooterModes.isPassEnabled());

    unloadHopperForPassingTrigger.onTrue(
        Commands.parallel(
            Commands.runOnce(() -> hopper.setKickerVelocity(shooter.getFlywheelLeadVelocity())),
            Commands.runOnce(hopper::spinFuelIntoKicker)));
    unloadHopperForPassingTrigger.onFalse(
        Commands.parallel(
            Commands.runOnce(hopper::stopKicker), Commands.runOnce(hopper::stopSpindexer)));

    // FIXME: what about passing when we are in the opposing alliance zone? When we cover this case,
    // we need to be careful that we don't exclude when we are near the opposing alliance's hub but
    // in the neutral zone in which case it won't block our pass.
    Trigger tooCloseToHubForPassTrigger =
        new Trigger(
            () ->
                shooterModes.isPassEnabled()
                    && Math.abs(
                            FieldConstants.Hub.innerCenterPoint.getMeasureX().in(Meters)
                                - swerveDrivetrain.getPose().getX())
                        < PASS_ZONE_TOLERANCE_X
                    && Math.abs(
                            FieldConstants.Hub.innerCenterPoint.getMeasureY().in(Meters)
                                - swerveDrivetrain.getPose().getY())
                        < PASS_ZONE_TOLERANCE_Y);

    // FIXME: add method that stops / starts the spindexer and kicker together instead of having to
    // do both of these commands every time we want to start or stop the hopper from feeding the
    // shooter
    tooCloseToHubForPassTrigger.onTrue(
        Commands.parallel(
            Commands.runOnce(hopper::stopKicker), Commands.runOnce(hopper::stopSpindexer)));
    tooCloseToHubForPassTrigger.onFalse(
        Commands.parallel(
            Commands.runOnce(() -> hopper.setKickerVelocity(shooter.getFlywheelLeadVelocity())),
            Commands.runOnce(hopper::spinFuelIntoKicker)));
  }
}
