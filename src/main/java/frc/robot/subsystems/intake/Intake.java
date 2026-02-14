package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

  private Distance deployerLinearPosition = Meters.of(0);

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final CurrentSpikeDetector rollerJamDetector =
      new CurrentSpikeDetector(
          IntakeConstants.ROLLER_JAMMED_CURRENT_AMPS,
          IntakeConstants.ROLLER_JAMMED_TIME_THRESHOLD_SECONDS);

  private final Alert rollerJamAlert = new Alert("Intake roller jammed", Alert.AlertType.kError);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Intake/TestingMode", 0);
  private final LoggedTunableNumber rollerVelocityRPS =
      new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0);
  private final LoggedTunableNumber rollerCurrent =
      new LoggedTunableNumber("Intake/RollerCurrent", 0.0);

  private final LoggedTunableNumber deployerPositionRotations =
      new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);

  private final LoggedTunableNumber deployerPositionLinearInches =
      new LoggedTunableNumber("Intake/DeployerPositionLinearInches", 0.0);

  private final LoggedTunableNumber deployerVoltage =
      new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);

  private final Debouncer deployerDeployedDebouncer = new Debouncer(0.2);
  private final Debouncer deployerRetractedDebouncer = new Debouncer(0.2);
  private final Debouncer rollerAtSetPointDebouncer = new Debouncer(0.1);

  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> intakeIO.setRollerCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps

  private final SysIdRoutine deployerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> intakeIO.setDeployerVoltage(output), null, this));

  public Intake(IntakeIO io) {
    this.intakeIO = io;

    SysIdRoutineChooser.getInstance().addOption("Roller Current", rollerSysIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Deployer Voltage", deployerSysIdRoutine);

    // Register System Check
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {

      if (deployerVoltage.get() != 0) {
        intakeIO.setDeployerVoltage(Volts.of(deployerVoltage.get()));
      } else if (deployerPositionRotations.get() != 0) {
        intakeIO.setDeployerPosition(Rotations.of(deployerPositionRotations.get()));
      } else if (deployerPositionLinearInches.get() != 0) {
        setLinearPosition(Inches.of(deployerPositionLinearInches.get()));
      }

      if (rollerCurrent.get() != 0) {
        intakeIO.setRollerCurrent(Amps.of(rollerCurrent.get()));
      } else if (rollerVelocityRPS.get() != 0) {
        intakeIO.setRollerVelocity(RotationsPerSecond.of(rollerVelocityRPS.get()));
      }
    }

    checkRollerJam();

    this.deployerLinearPosition =
        DEPLOYER_CIRCUMFERENCE.times(inputs.deployerAngularPosition.in(Rotations));

    Logger.recordOutput(SUBSYSTEM_NAME + "/DeployerLinearPosition", deployerLinearPosition);
    LoggedTracer.record("Intake");
  }

  public void setLinearPosition(Distance linearPosition) {

    Angle angularPosition = Rotations.of(linearPosition.div(DEPLOYER_CIRCUMFERENCE).magnitude());

    intakeIO.setDeployerPosition(angularPosition);
  }

  private void checkRollerJam() {
    if (rollerJamDetector.update(Math.abs(inputs.rollerStatorCurrent.in(Amps)))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                      Commands.runOnce(() -> intakeIO.setRollerCurrent(Amps.of(0.0)), this),
                      Commands.run(
                              () -> LEDs.getInstance().requestState(LEDs.States.ELEVATOR_JAMMED))
                          .withTimeout(1.0))
                  .withName("Stop Intake Jammed"));
      rollerJamAlert.set(true);
    } else {
      rollerJamAlert.set(false);
    }
  }

  public void startRoller() {
    intakeIO.setRollerVelocity(IntakeConstants.ROLLER_TARGET_VELOCITY);
  }

  public void stopRoller() {
    intakeIO.setRollerVelocity(RotationsPerSecond.of(0.0));
  }

  public void outTakeRoller() {
    intakeIO.setRollerVelocity(IntakeConstants.ROLLER_EJECT_VELOCITY);
  }

  public void deployIntake() {
    setLinearPosition(DEPLOYED_LINEAR_POSITION);
  }

  public void retractIntake() {
    setLinearPosition(RETRACTED_LINEAR_POSITION);
  }

  public boolean isRollerAtSetpoint(AngularVelocity velocity) {
    return rollerAtSetPointDebouncer.calculate(
        inputs.rollerVelocity.isNear(velocity, IntakeConstants.ROLLER_VELOCITY_TOLERANCE));
  }

  public boolean isDeployerAtSetpoint(Distance linearDistance) {
    return deployerDeployedDebouncer.calculate(
        this.deployerLinearPosition.isNear(linearDistance, DEPLOYER_LINEAR_POSITION_TOLERANCE));
  }

  public boolean isDeployed() {
    return deployerDeployedDebouncer.calculate(
        this.deployerLinearPosition.isNear(
            DEPLOYED_LINEAR_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE));
  }

  public boolean isRetracted() {
    return deployerRetractedDebouncer.calculate(
        this.deployerLinearPosition.isNear(
            RETRACTED_LINEAR_POSITION, DEPLOYER_LINEAR_POSITION_TOLERANCE));
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> intakeIO.setRollerVelocity(RotationsPerSecond.of(10))),
            Commands.waitSeconds(0.5)
                .andThen(Commands.waitUntil(() -> isRollerAtSetpoint(RotationsPerSecond.of(10)))),
            Commands.runOnce(
                () -> {
                  if (inputs.rollerVelocity.lt(RotationsPerSecond.of(8))) {
                    FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME, "Roller failed to reach 10 RPS in System Check");
                  }
                }),
            Commands.runOnce(this::stopRoller),
            Commands.runOnce(() -> intakeIO.setDeployerPosition(Rotations.of(0.1))),
            Commands.waitSeconds(0.5)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            isDeployerAtSetpoint(
                                Inches.of(0.1 * DEPLOYER_CIRCUMFERENCE.in(Inches))))),
            Commands.runOnce(
                () -> {
                  if (!inputs.deployerAngularPosition.isNear(
                      Rotations.of(0.1), Rotations.of(0.02))) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME, "Deployer failed to reach 0.1 Rot in System Check");
                  }
                }),
            Commands.runOnce(this::retractIntake),
            Commands.waitSeconds(0.5).andThen(Commands.waitUntil(this::isRetracted)))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(
            Commands.runOnce(
                () -> {
                  stopRoller();
                  intakeIO.setDeployerPosition(inputs.deployerAngularPosition);
                }));
  }
}
