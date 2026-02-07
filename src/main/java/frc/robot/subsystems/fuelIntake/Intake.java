package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private CurrentSpikeDetector rollerJamDetector =
      new CurrentSpikeDetector(
          IntakeConstants.ROLLER_JAMMED_CURRENT_AMPS,
          IntakeConstants.ROLLER_JAMMED_TIME_THRESHOLD_SECONDS);

  private Alert rollerJamAlert = new Alert("Intake roller jammed", Alert.AlertType.kError);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Intake/TestingMode", 0);
  private final LoggedTunableNumber rollerVelocityRPS =
      new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0);
  private final LoggedTunableNumber rollerVoltage =
      new LoggedTunableNumber("Intake/RollerVoltage", 0.0);
  private final LoggedTunableNumber rollerRampRate =
      new LoggedTunableNumber("Intake/RollerRampRate", 0.0);
  private final LoggedTunableNumber rollerStepCurrent =
      new LoggedTunableNumber("Intake/RollerStepCurrent", 0.0);
  private final LoggedTunableNumber deployerPositionRotations =
      new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);
  private final LoggedTunableNumber deployerVoltage =
      new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);
  private final LoggedTunableNumber deployerRampRate =
      new LoggedTunableNumber("Intake/DeployerRampRate", 0.0);
  private final LoggedTunableNumber deployerStepVoltage =
      new LoggedTunableNumber("Intake/DeployerStepVoltage", 0.0);

  private final Debouncer deployerDeployedDebouncer = new Debouncer(0.5);
  private final Debouncer deployerRetractedDebouncer = new Debouncer(0.5);

  // The SysId routine is used to characterize the mechanism. While the SysId routine is intended to
  // be used for voltage control, we can apply a current instead and reinterpret the units when
  // performing the analysis in SysId.

  private SysIdRoutine rollerSysIdRoutine;
  private SysIdRoutine deployerSysIdRoutine;

  public Intake(IntakeIO io) {
    this.io = io;

    rollerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(rollerRampRate.get()).per(Second), // will actually be a ramp rate of x A/s
                Volts.of(rollerStepCurrent.get()), // will actually be a step to x A
                null, // Use default timeout (10 s)
                state ->
                    SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.setRollerCurrent(Amps.of(output.in(Volts))),
                null,
                this)); // treat volts as amps

    deployerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(deployerRampRate.get()).per(Second), // override default ramp rate (1 V/s)
                Volts.of(deployerStepVoltage.get()), // override default step voltage (7 V)
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(io::setDeployerVoltage, null, this));

    SysIdRoutineChooser.getInstance().addOption("Roller Current", rollerSysIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Deployer Voltage", deployerSysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Calculate linear position from angular position
    Distance linearPosition =
        IntakeConstants.DEPLOYER_CIRCUMFERENCE.times(inputs.angularPosition.in(Rotations));

    Logger.processInputs(IntakeConstants.SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      io.setRollerVelocity(RotationsPerSecond.of(rollerVelocityRPS.get()));
      io.setRollerVoltage(Volts.of(rollerVoltage.get()));
      io.setDeployerVoltage(Volts.of(deployerVoltage.get()));
      io.setDeployerPosition(Rotations.of(deployerPositionRotations.get()));
    }

    if (rollerJamDetector.update(Math.abs(inputs.rollerStatorCurrentAmps.in(Amps)))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                      Commands.runOnce(() -> io.setRollerVoltage(Volts.of(0.0)), this),
                      Commands.run(
                              () ->
                                  LEDs.getInstance()
                                      .requestState(
                                          LEDs.States
                                              .ELEVATOR_JAMMED)) // LED not made for intake jamming
                          // yet
                          .withTimeout(1.0))
                  .withName("stop intake jammed"));
      rollerJamAlert.set(true);
    } else {
      rollerJamAlert.set(false);
    }

    LoggedTracer.record("Intake");
  }

  public void startRoller() {
    io.setRollerVelocity(IntakeConstants.INTAKE_ROLLER_VELOCITY);
  }

  public void stopRoller() {
    io.setRollerVelocity(RotationsPerSecond.of(0.0));
  }

  public void reverseRoller() {
    io.setRollerVelocity(IntakeConstants.INTAKE_ROLLER_VELOCITY.unaryMinus());
  }

  public void deployIntake() {
    Angle angularSetpoint =
        Rotations.of(
            IntakeConstants.DEPLOYED_POSITION
                .div(IntakeConstants.DEPLOYER_CIRCUMFERENCE)
                .magnitude());
    io.setDeployerPosition(angularSetpoint);
  }

  public void retractIntake() {
    Angle angularSetpoint =
        Rotations.of(
            IntakeConstants.RETRACTED_POSITION
                .div(IntakeConstants.DEPLOYER_CIRCUMFERENCE)
                .magnitude());
    io.setDeployerPosition(angularSetpoint);
  }

  public Distance getDeployerPosition() {
    return IntakeConstants.DEPLOYER_CIRCUMFERENCE.times(inputs.angularPosition.in(Rotations));
  }

  public boolean isDeployed(Positions position) {
    return deployerDeployedDebouncer.calculate(
        getDeployerPosition()
            .isNear(
                IntakeConstants.DEPLOYED_POSITION, IntakeConstants.DEPLOYED_POSITION_TOLERANCE));
  }

  public boolean isRetracted() {
    return deployerRetractedDebouncer.calculate(
        getDeployerPosition()
            .isNear(
                IntakeConstants.RETRACTED_POSITION, IntakeConstants.RETRACTED_POSITION_TOLERANCE));
  }
}
