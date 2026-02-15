package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Climber/TestingMode", 0);
  private final LoggedTunableNumber climberAngleDegrees =
      new LoggedTunableNumber("Climber/AngleDegrees", MIN_ANGLE_DEGREES.in(Degrees));
  private final LoggedTunableNumber climberVoltage =
      new LoggedTunableNumber("Climber/Voltage", 0.0);

  private final Debouncer isAtTargetDebouncer = new Debouncer(0.5);

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setClimberVoltage(output), null, this));

  public Climber(ClimberIO io) {
    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Climber Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      if (climberAngleDegrees.get() != MIN_ANGLE_DEGREES.in(Degrees)) {
        io.setClimberAngle(Degrees.of(climberAngleDegrees.get()));
      } else if (climberVoltage.get() != 0) {
        io.setClimberVoltage(Volts.of(climberVoltage.get()));
      }
    }

    LoggedTracer.record("Climber");
  }

  public void setClimberAngle(Angle angle) {
    io.setClimberAngle(angle);
  }

  public void setClimberVoltage(Voltage voltage) {
    io.setClimberVoltage(voltage);
  }

  public boolean isAngleAtSetpoint() {

    return isAtTargetDebouncer.calculate(
        inputs.referenceAngle.isNear(inputs.climberAngle, ANGLE_TOLERANCE_DEGREES));
  }

  public Angle getAngle() {
    return inputs.climberAngle;
  }

  public void stop() {
    io.setClimberVoltage(Volts.of(0));
  }
}
