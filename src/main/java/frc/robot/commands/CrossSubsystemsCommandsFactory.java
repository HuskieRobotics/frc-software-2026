package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
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

    // normal stop and shoot command, triggered by the right trigger (rotate 1)
    // this can be run in any mode, and will use the typical stop and shoot
    // if we are in a shoot on the move mode, it will let the hopper keep running to feed fuel into
    // the shooter
    oi.getManualShootButton()
        .onTrue(
            getStopAndShootCommand(oi, swerveDrivetrain, shooter, hopper, intake, shooterModes));

    // return to snake driving (probably unnecessary since we don't xstance anymore)
    oi.getManualShootButton()
        .onFalse(
            Commands.parallel(
                    Commands.either(
                        getSnakeDriveCommand(oi, swerveDrivetrain),
                        SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(
                            oi, swerveDrivetrain),
                        oi.getSnakeDriveButton()),
                    Commands.runOnce(intake::getDeployAndStartCommand, intake),
                    Commands.runOnce(hopper::stop, hopper))
                .withName("stop shooting"));

    // this is bound to the left trigger (translate 1)
    // this does a typical shot but starts the intake jostle immediately instead of
    // waiting for 10 balls to pass through
    // this also has the option to drive to bank, but will likely be deprecated
    oi.getForceSafeShootButton()
        .onTrue(
            Commands.either(
                getScoreSafeShotCommand(
                    oi, swerveDrivetrain, shooter, hopper, intake, shooterModes),
                getForceShootCommand(oi, shooter, hopper, intake, shooterModes),
                () ->
                    (shooterModes.isManualShootEnabled()
                        || shooterModes.isLockedShooterEnabled())));

    // return to driving normally (probably unnecessary since we don't xstance anymore)                  
    oi.getForceSafeShootButton()
        .onFalse(
            Commands.parallel(
                    Commands.either(
                        getSnakeDriveCommand(oi, swerveDrivetrain),
                        SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(
                            oi, swerveDrivetrain),
                        oi.getSnakeDriveButton()),
                    Commands.runOnce(intake::getDeployAndStartCommand, intake),
                    Commands.runOnce(hopper::stop, hopper))
                .withName("stop force-shooting"));

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
            () -> {
              if (Math.hypot(oi.getTranslateX(), oi.getTranslateY()) > 0.06) {
                return Optional.of(new Rotation2d(oi.getTranslateX(), oi.getTranslateY()));
              } else {
                return Optional.empty();
              }
            })
        .withName("Snake Drive Command");
  }

  public static Command getStopAndShootCommand(
      OperatorInterface oi,
      SwerveDrivetrain drivetrain,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShooterModes shooterModes) {
    return Commands.repeatingSequence(
            Commands.parallel(
                    // let the hopper continue to do its thing if we are in shoot on the move mode
                    Commands.either(
                        Commands.none(),
                        hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocity),
                        () ->
                            (shooterModes.isShootOnTheMoveEnabled()
                                || shooterModes.isPassOnTheMoveEnabled())),
                    getJostleCommand(intake, shooter),
                    Commands.either(
                        Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                        Commands.run(() -> LEDs.getInstance().requestState(States.SHOOTING)),
                        () ->
                            shooterModes.isManualPassEnabled()
                                || shooterModes.isPassOnTheMoveEnabled()))
                .until(shooterModes::isTurretNotNearSetPoint)
                .andThen(
                    // let the hopper continue to run if shooting on the move
                    Commands.either(
                        Commands.none(),
                        Commands.runOnce(hopper::stop, hopper),
                        () ->
                            (shooterModes.isShootOnTheMoveEnabled()
                                || shooterModes.isPassOnTheMoveEnabled()))))
        .withName("stop and shoot or pass");
  }

  public static Command getForceShootCommand(
      OperatorInterface oi,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShooterModes shooterModes) {
    return Commands.repeatingSequence(
            Commands.parallel(
                    // let the hopper continue to do its thing if we are in shoot on the move mode
                    Commands.either(
                        Commands.none(),
                        hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocity),
                        () ->
                            (shooterModes.isShootOnTheMoveEnabled()
                                || shooterModes.isPassOnTheMoveEnabled())),
                    getUnrestrictedJostleCommand(intake, shooter),
                    Commands.either(
                        Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                        Commands.run(() -> LEDs.getInstance().requestState(States.SHOOTING)),
                        () ->
                            shooterModes.isManualPassEnabled()
                                || shooterModes.isPassOnTheMoveEnabled()))
                .until(shooterModes::isTurretNotNearSetPoint)
                .andThen(
                    // let the hopper continue to run if shooting on the move
                    Commands.either(
                        Commands.none(),
                        Commands.runOnce(hopper::stop, hopper),
                        () ->
                            (shooterModes.isShootOnTheMoveEnabled()
                                || shooterModes.isPassOnTheMoveEnabled()))))
        .withName("force shoot or pass");
  }

  private static Command getJostleCommand(Intake intake, Shooter shooter) {
    return Commands.sequence(
            Commands.runOnce(shooter::resetFuelCount),
            Commands.waitUntil(() -> shooter.getFuelCount() >= JOSTLE_INITIAL_FUEL_COUNT),
            Commands.repeatingSequence(
                Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_RETRACTED_POSITION)),
                Commands.waitUntil(
                    () ->
                        intake
                            .getPosition()
                            .isNear(JOSTLE_RETRACTED_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE)),
                Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_EXTENDED_POSITION)),
                Commands.waitUntil(
                    () ->
                        intake
                            .getPosition()
                            .isNear(JOSTLE_EXTENDED_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE)),
                Commands.waitSeconds(0.5)))
        .withName("Jostle");
  }

  private static Command getUnrestrictedJostleCommand(Intake intake, Shooter shooter) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_RETRACTED_POSITION)),
            Commands.waitUntil(
                () ->
                    intake
                        .getPosition()
                        .isNear(JOSTLE_RETRACTED_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE)),
            Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_EXTENDED_POSITION)),
            Commands.waitUntil(
                () ->
                    intake
                        .getPosition()
                        .isNear(JOSTLE_EXTENDED_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE)),
            Commands.waitSeconds(0.5))
        .withName("Unrestricted Jostle");
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
    Trigger unloadHopperOnTheMoveTrigger =
        new Trigger(
                () ->
                    shooterModes.isShootOnTheMoveEnabled() || shooterModes.isPassOnTheMoveEnabled())
            .and(DriverStation::isTeleopEnabled);
    unloadHopperOnTheMoveTrigger.onTrue(
        Commands.repeatingSequence(
                Commands.parallel(
                        hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocity),
                        Commands.repeatingSequence(
                            Commands.either(
                                Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                                Commands.run(
                                    () -> LEDs.getInstance().requestState(States.SHOOTING)),
                                shooterModes::isPassOnTheMoveEnabled)))
                    .until(shooterModes::isTurretNotNearSetPoint)
                    .andThen(Commands.runOnce(hopper::stop, hopper)))
            .withName("feed fuel"));
    unloadHopperOnTheMoveTrigger.onFalse(Commands.runOnce(hopper::stop, hopper));
  }
}
