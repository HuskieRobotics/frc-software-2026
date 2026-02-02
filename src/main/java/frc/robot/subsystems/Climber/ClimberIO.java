package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
    public boolean connected = false;
    public Voltage voltageSupplied = Voltage.of(0.0);
    public Current statorCurrent = 0.0;
    public Current supplyCurrent = 0.0;
    public Temperature temperature = Temperature.of(0.0);
    public double positionRotations = 0.0;
    public double positionInches = inches.of(0.0);

    public Angle closedLoopError = Rotations.of(0.0);
    public Angle closedLoopReference = Rotations.of(0.0);

    public Distance linearPosition = Meters.of(0.0);
  }

  public default void setPosition(Distance position) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
}
