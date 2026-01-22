package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.JAMMED_CURRENT_AMPS;
import static frc.robot.subsystems.elevator.ElevatorConstants.JAMMED_TIME_THRESHOLD_SECONDS;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private CurrentSpikeDetector rollerJamDetector =
      new CurrentSpikeDetector(JAMMED_CURRENT_AMPS, JAMMED_TIME_THRESHOLD_SECONDS);

  private Alert rollerJamAlert =
      new Alert("Intake Roller Jammed 💀, use manual control.", Alert.AlertType.kError);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Intake/TestingMode", 0);
  private final LoggedTunableNumber rollerVelocityRPS =
      new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0);
  private final LoggedTunableNumber deployerPositionRotations =
      new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);
  private final LoggedTunableNumber deployerVoltage =
      new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);
  private final LoggedTunableNumber rollerVoltage =
      new LoggedTunableNumber("Intake/RollerVoltage", 0.0);
  private final LoggedTunableNumber deployerCurrent =
      new LoggedTunableNumber("Intake/DeployerCurrent", 0.0);

  private final Debouncer deployerDeployedDebouncer = new Debouncer(0.5);
  private final Debouncer deployerRetractedDebouncer = new Debouncer(0.5);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setRollerVoltage(output), null, this));

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void startRoller() {
    io.setRollerVelocity(IntakeConstants.INTAKE_ROLLER_VELOCITY_RPS);
  }

  public void stopRoller() {
    io.setRollerVelocity(0.0);
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
    } else if (inputs.linearPosition.toRotations() >= IntakeConstants.DEPLOYED_POSITION_ROTATIONS) {
      return deployerDeployedDebouncer.calculate(true);
    } else {
      return deployerDeployedDebouncer.calculate(false);
    }
  }

  public boolean isRetracted() {
    // Only consider retracted if the hardware is connected
    if (!inputs.deployerConnected) {
      return false;
    } else if (inputs.linearPosition.toRotations()
        <= IntakeConstants.RETRACTED_POSITION_ROTATIONS) {
      return deployerRetractedDebouncer.calculate(true);
    } else {
      return deployerRetractedDebouncer.calculate(false);
    }
  }
}
