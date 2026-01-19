package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

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

        // Voltage
        Voltage rollerVoltageSupplied = Volts.of(0.0);
        Voltage spindexerVoltageSupplied = Volts.of(0.0);

        // Temp 
        Temperature rollerTemperatureCelsius = Celsius.of(0.0);
        Temperature spindexerTemperatureCelsius = Celsius.of(0.0);

        // Connections
        boolean rollerMotorConnected = false;
        boolean spindexerMotorConnected = false;

        // Stator 
        Current spindexerStatorCurrentAmps = Amps.of(0.0);
        Current rollerStatorCurrentAmps = Amps.of(0.0);

        // Supply
        Current spindexerSupplyCurrentAmps = Amps.of(0.0);
        Current rollerSupplyCurrentAmps = Amps.of(0.0);


    }

    public default void setRollerVoltage(double voltage) {}
    public default void setSpindexerVoltage(double voltage) {}

    public default void updateInputs(HopperIOInputs inputs) {}


}
