package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {

    boolean connected = false;
    
    Voltage voltageSupplied = Volts.of(0.0);
    Current statorCurrent = Amps.of(0.0);
    Current supplyCurrent = Amps.of(0.0);
    Temperature temperature = Celsius.of(0.0);
    Angle positionRotations = Rotations.of(0.0);
    Angle referenceAngle = Rotations.of(0.0);
    Angle climberAngle = Degrees.of(0.0);
    Angle closedLoopError = Rotations.of(0.0);
    Angle closedLoopReference = Rotations.of(0.0);
}

  public default void updateInputs(ClimberIOInputs inputs) {}

   public default void setPosition(Angle angle) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
}
