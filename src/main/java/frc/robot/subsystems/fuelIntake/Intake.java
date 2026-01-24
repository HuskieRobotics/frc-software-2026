package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private CurrentSpikeDetector rollerJamDetector =
      new CurrentSpikeDetector(
          IntakeConstants.ROLLER_JAMMED_CURRENT_AMPS,
          IntakeConstants.ROLLER_JAMMED_TIME_THRESHOLD_SECONDS);

  private Alert rollerJamAlert = new Alert("Intake roller jammed 💀", Alert.AlertType.kError);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Intake/TestingMode", 0);
  private final LoggedTunableNumber rollerVelocityRPS =
      new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0);
  private final LoggedTunableNumber rollerCurrent =
      new LoggedTunableNumber("Intake/RollerCurrent", 0.0);
  private final LoggedTunableNumber deployerPositionRotations =
      new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);
  private final LoggedTunableNumber deployerVoltage =
      new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);
  private final LoggedTunableNumber deployerCurrent =
      new LoggedTunableNumber("Intake/DeployerCurrent", 0.0);

  private final Debouncer deployerDeployedDebouncer = new Debouncer(0.5);
  private final Debouncer deployerRetractedDebouncer = new Debouncer(0.5);

  private SysIdRoutine rollerSysIdRoutine;
  private SysIdRoutine deployerSysIdRoutine;

  public Intake(IntakeIO io) {
    this.io = io;

    rollerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // Use default timeout (10 s
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(io::setRollerVoltage, null, this));

    deployerSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // Use default timeout (10 s
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(io::setDeployerVoltage, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(IntakeConstants.SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      io.setRollerVelocity(rollerVelocityRPS.get());
      io.setRollerCurrent(Amps.of(rollerCurrent.get()));
      io.setDeployerVoltage(Volts.of(deployerVoltage.get()));
      io.setDeployerCurrent(Amps.of(deployerCurrent.get()));
    }

    if (rollerJamDetector.update(Math.abs(inputs.rollerStatorCurrentAmps.in(Amps)))) {
      reverseRoller();
      rollerJamAlert.set(true);
    } else {
      rollerJamAlert.set(false);
    }

    LoggedTracer.record("Intake");
  }

  public void startRoller() {
    io.setRollerVelocity(IntakeConstants.INTAKE_ROLLER_VELOCITY_RPS);
  }

  public void stopRoller() {
    io.setRollerVelocity(0.0);
  }

  public void reverseRoller() {
    io.setRollerVelocity(-IntakeConstants.INTAKE_ROLLER_VELOCITY_RPS);
  }

  public void deployIntake() {
    io.setDeployerPosition(IntakeConstants.DEPLOYED_POSITION_ROTATIONS);
  }

  public void retractIntake() {
    io.setDeployerPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
  }

  public boolean isDeployed() {
    // Only consider deployed if the hardware is connected
    if (!inputs.deployerConnected) {
      return false;
    } else if (inputs.linearPosition.in(Meters) >= deployerPositionRotations.get()) {
      return deployerDeployedDebouncer.calculate(true);
    } else {
      return deployerDeployedDebouncer.calculate(false);
    }
  }

  public boolean isRetracted() {
    // Only consider retracted if the hardware is connected
    if (!inputs.deployerConnected) {
      return false;
    } else if (inputs.linearPosition.in(Meters) <= 0.0) {
      return deployerRetractedDebouncer.calculate(true);
    } else {
      return deployerRetractedDebouncer.calculate(false);
    }
  }
}
