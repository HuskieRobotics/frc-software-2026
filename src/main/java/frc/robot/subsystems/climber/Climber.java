package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
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

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getClimberSystemCheckCommand());
  }

  public Command getClimberSystemCheckCommand() {

    return Commands.sequence(
        Commands.runOnce(() -> io.setClimberAngle(CLIMBER_ANGLE_SETPOINT_1)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> io.checkPosition(CLIMBER_ANGLE_SETPOINT_1)),
        Commands.runOnce(() -> io.setClimberAngle(CLIMBER_ANGLE_SETPOINT_2)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> io.checkPosition(CLIMBER_ANGLE_SETPOINT_2)),
        Commands.runOnce(() -> io.setClimberAngle(CLIMBER_ANGLE_SETPOINT_3)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> io.checkPosition(CLIMBER_ANGLE_SETPOINT_3)));
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

  public Angle getClimberAngle() {
    return inputs.climberAngle;
  }

  public void setClimberVoltage(Voltage voltage) {
    io.setClimberVoltage(voltage);
  }

  public void setPosition(Angle angleDegrees) {
    io.setClimberAngle(angleDegrees);
  }

  public boolean isAtPosition(Angle targetAngle) {
    return inputs.referenceAngle.isNear(inputs.climberAngle, ANGLE_TOLERANCE_DEGREES);
  }

  public void checkPosition(double intendedPositionDegrees) {
    if (Math.abs(getAngle().in(Degrees) - intendedPositionDegrees)
        > ClimberConstants.ANGLE_TOLERANCE_DEGREES.in(Degrees)) {
      if (getAngle().in(Degrees) - intendedPositionDegrees < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Climber Angle is too low, should be "
                    + intendedPositionDegrees
                    + " but is "
                    + getAngle().in(Degrees));
      } else if (getAngle().in(Degrees) - intendedPositionDegrees > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Climber Angle is too high, should be "
                    + intendedPositionDegrees
                    + " but is "
                    + getAngle().in(Degrees));
      } else {
        FaultReporter.getInstance()
            .addFault(SUBSYSTEM_NAME, "Climber Angle is at its intended position");
      }
    }
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
