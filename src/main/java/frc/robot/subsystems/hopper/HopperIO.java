package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class HopperIOInputs {

    // Connections
    boolean kickerMotorConnected = false;
    boolean spindexerMotorConnected = false;

    // Voltage
    Voltage kickerVoltageSupplied = Volts.of(0.0);
    Voltage spindexerVoltageSupplied = Volts.of(0.0);

    // Temp
    Temperature kickerTemperatureCelsius = Celsius.of(0.0);
    Temperature spindexerTemperatureCelsius = Celsius.of(0.0);

    // Stator
    Current spindexerStatorCurrent = Amps.of(0.0);
    Current kickerStatorCurrent = Amps.of(0.0);

    // Supply
    Current spindexerSupplyCurrent = Amps.of(0.0);
    Current kickerSupplyCurrent = Amps.of(0.0);

    // Angular Velocity
    AngularVelocity spindexerVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity kickerVelocity = RotationsPerSecond.of(0.0);

    AngularVelocity spindexerReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity kickerReferenceVelocity = RotationsPerSecond.of(0.0);

    AngularVelocity closedLoopErrorSpindexer = RotationsPerSecond.of(0.0);
    AngularVelocity closedLoopErrorKicker = RotationsPerSecond.of(0.0);

    AngularVelocity closedLoopReferenceVelocitySpindexer = RotationsPerSecond.of(0.0);
    AngularVelocity closedLoopReferenceVelocityKicker = RotationsPerSecond.of(0.0);
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setSpindexerVelocity(AngularVelocity velocity) {}

  public default void setKickerVelocity(AngularVelocity velocity) {}

  public default void setSpindexerCurrent(Current amps) {}

  public default void setKickerCurrent(Current amps) {}
}
