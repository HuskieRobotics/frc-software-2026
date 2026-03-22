package frc.robot.commands;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.MathUtils;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
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
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocityMPS());
  private static final LoggedTunableNumber driveToPoseMaxAcceleration =
      new LoggedTunableNumber(
          "DriveToBank/DriveToPoseMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseDriveMaxAccelerationMPSPS());

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
              RobotConfig.getInstance().getDriveToPoseTurnMaxVelocityRPS(),
              RobotConfig.getInstance().getDriveToPoseTurnMaxAccelerationRPSPS()));

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi,
      SwerveDrivetrain swerveDrivetrain,
      Intake intake,
      Hopper hopper,
      Shooter shooter,
      ShooterModes shooterModes) {

    oi.getClearAllFaults()
        .onTrue(FaultReporter.getInstance().getClearAllFaultsCommand().ignoringDisable(true));
    oi.getCheckForFaults()
        .onTrue(FaultReporter.getInstance().getCheckForFaultsCommand().ignoringDisable(true));

    configureCrossSubsystemsTriggers(shooterModes, shooter, hopper);

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(swerveDrivetrain, intake, hopper, shooter, oi));

    /**
     * Behavior of the shoot buttons (manual shoot and force shoot) is as follows for each of the
     * shooter modes. The difference between the manual shoot button and the force shoot button is
     * that the manual shoot button waits for 10 fuel to pass through the shooter before jostling
     * the intake while the force shoot button immediately jostles the intake.
     *
     * <p>MANUAL_SHOOT: the hooper starts running when either button is pressed and stops when
     * either button is released. The jostling behavior is determined based on which button is
     * pressed.
     *
     * <p>SHOOT_OTM: the hopper is running before either button is pressed and continues to run
     * after either button is released. The buttons just impact how the intake jostles the fuel.
     *
     * <p>COLLECT_AND_HOLD: pressing either button has no effect
     *
     * <p>NEAR_TRENCH: pressing either button has no effect
     *
     * <p>MANUAL_PASS: the hooper starts running when either button is pressed and stops when either
     * button is released. The jostling behavior is determined based on which button is pressed.
     *
     * <p>PASS_OTM: the hopper is running before either button is pressed and continues to run after
     * either button is released. The buttons just impact how the intake jostles the fuel.
     *
     * <p>SHOOTER_LOCKED: the hooper starts running when either button is pressed and stops when
     * either button is released. The jostling behavior is determined based on which button is
     * pressed.
     */

    // normal stop and shoot command, triggered by the right trigger (rotate 1)
    // this can be run in any mode, and will use the typical stop and shoot
    // if we are in a shoot on the move mode, it will let the hopper keep running to feed fuel into
    // the shooter
    oi.getManualShootButton()
        .and(() -> !shooterModes.isCollectAndHoldEnabled() && !shooterModes.isNearTrenchEnabled())
        .whileTrue(
            Commands.either(
                getStopAndShootCommand(
                    shooter, hopper, intake, shooterModes, getJostleCommand(intake, shooter)),
                getStopAndShootCommand(
                    shooter, hopper, intake, shooterModes, getForceJostleCommand(intake)),
                shooterModes::isManualShootEnabled));

    // this is bound to the left trigger (translate 1)
    // this does a typical shot but starts the intake jostle immediately instead of
    // waiting for 10 balls to pass through
    // this also has the option to drive to bank, but will likely be deprecated
    oi.getForceSafeShootButton()
        .and(() -> !shooterModes.isCollectAndHoldEnabled() && !shooterModes.isNearTrenchEnabled())
        .whileTrue(
            getStopAndShootCommand(
                shooter, hopper, intake, shooterModes, getForceJostleCommand(intake)));

    oi.getSnakeDriveButton().toggleOnTrue(getSnakeDriveCommand(oi, swerveDrivetrain));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    oi.getIncrementFlywheelVelocityButton()
        .onTrue(Commands.runOnce(shooterModes::incrementShotVelocity));
    oi.getDecrementFlywheelVelocityButton()
        .onTrue(Commands.runOnce(shooterModes::decrementShotVelocity));
    oi.getMoveTurretLeftButton().onTrue(Commands.runOnce(shooterModes::moveTurretOneDegreeLeft));
    oi.getMoveTurretRightButton().onTrue(Commands.runOnce(shooterModes::moveTurretOneDegreeRight));

    registerSysIdCommands(oi);
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
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      ShooterModes shooterModes,
      Command jostleCommand) {
    return Commands.repeatingSequence(
            Commands.parallel(
                    // let the hopper continue to do its thing if we are in shoot on the move mode
                    hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocityRPS),
                    jostleCommand,
                    Commands.repeatingSequence(
                        Commands.either(
                            Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                            Commands.run(() -> LEDs.getInstance().requestState(States.SHOOTING)),
                            () ->
                                shooterModes.isManualPassEnabled()
                                    || shooterModes.isPassOnTheMoveEnabled())))
                .until(shooterModes::isTurretNotNearSetPoint)
                .andThen(
                    Commands.sequence(
                        Commands.runOnce(hopper::stop, hopper),
                        Commands.runOnce(
                            () -> LEDs.getInstance().requestState(States.TURRET_NOT_AT_SETPOINT)))))
        .finallyDo(
            () -> {
              CommandScheduler.getInstance().schedule(intake.getDeployAndStartCommand());
              if (shooterModes.isShootOnTheMoveEnabled() || shooterModes.isPassOnTheMoveEnabled()) {
                CommandScheduler.getInstance()
                    .schedule(getShootWhenAimedCommand(shooterModes, shooter, hopper));
              } else {
                hopper.stop();
              }
            })
        .withName("shoot or pass");
  }

  public static Command getJostleCommand(Intake intake, Shooter shooter) {
    return Commands.sequence(
            Commands.runOnce(shooter::resetFuelCount),
            Commands.waitUntil(() -> shooter.getFuelCount() >= JOSTLE_INITIAL_FUEL_COUNT)
                .withTimeout(2.0), // was 2.5
            getForceJostleCommand(intake))
        .withName("Jostle");
  }

  // starts the sequence by bringing the intake in to 5 inches
  // waits until we are there, then starts repeating sequence:
  //   wait 0.5s
  //   extend intake out to 11 inches
  //   retract intake back to 3.5 inches
  public static Command getForceJostleCommand(Intake intake) {
    return Commands.sequence(
            Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_FIRST_RETRACT_POSITION_METERS)),
            Commands.deadline(
                Commands.waitSeconds(0.75),
                Commands.waitUntil(
                    () ->
                        MathUtils.isNear(
                            intake.getPositionMeters(),
                            JOSTLE_FIRST_RETRACT_POSITION_METERS,
                            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS))),
            Commands.repeatingSequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> intake.setLinearPosition(JOSTLE_EXTENDED_POSITION_METERS)),
                Commands.waitUntil(
                    () ->
                        MathUtils.isNear(
                            intake.getPositionMeters(),
                            JOSTLE_EXTENDED_POSITION_METERS,
                            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS)),
                Commands.runOnce(
                    () -> intake.setLinearPosition(JOSTLE_SUBSEQUENT_RETRACT_POSITION_METERS)),
                Commands.deadline(
                    Commands.waitSeconds(0.75),
                    Commands.waitUntil(
                        () ->
                            MathUtils.isNear(
                                intake.getPositionMeters(),
                                JOSTLE_SUBSEQUENT_RETRACT_POSITION_METERS,
                                DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS)))))
        .withName("Force Jostle");
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

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(oi, drivetrain)
        .withName("Override driveToPose");
  }

  private static void configureCrossSubsystemsTriggers(
      ShooterModes shooterModes, Shooter shooter, Hopper hopper) {
    Trigger unloadHopperOnTheMoveTrigger =
        new Trigger(
                () ->
                    shooterModes.isShootOnTheMoveEnabled() || shooterModes.isPassOnTheMoveEnabled())
            .and(DriverStation::isTeleopEnabled);
    unloadHopperOnTheMoveTrigger.onTrue(getShootWhenAimedCommand(shooterModes, shooter, hopper));
    unloadHopperOnTheMoveTrigger.onFalse(Commands.runOnce(hopper::stop, hopper));
  }

  private static Command getShootWhenAimedCommand(
      ShooterModes shooterModes, Shooter shooter, Hopper hopper) {
    return Commands.repeatingSequence(
            Commands.parallel(
                    hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocityRPS),
                    Commands.repeatingSequence(
                        Commands.either(
                            Commands.run(() -> LEDs.getInstance().requestState(States.PASSING)),
                            Commands.run(() -> LEDs.getInstance().requestState(States.SHOOTING)),
                            shooterModes::isPassOnTheMoveEnabled)))
                .until(shooterModes::isTurretNotNearSetPoint)
                .andThen(
                    Commands.sequence(
                        Commands.runOnce(hopper::stop, hopper),
                        Commands.runOnce(
                            () -> LEDs.getInstance().requestState(States.TURRET_NOT_AT_SETPOINT)))))
        .withName("shoot when aimed");
  }
}
