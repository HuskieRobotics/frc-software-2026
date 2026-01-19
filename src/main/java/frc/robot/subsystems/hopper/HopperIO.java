package frc.robot.subsystems.hopper;

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
        double backVoltageSupplied = 0.0;
        double midVoltageSupplied = 0.0;

        // Temp FIXME: the following may not be needed
        // double backTemperatureCelsius = 0.0;
        // double midTemperatureCelsius = 0.0;

        // Connections
        boolean backMotorConnected = false;
        boolean midMotorConnected = false;


    }

    public default void setBackVoltage(double voltage) {}
    public default void setMidVoltage(double voltage) {}

    public default void updateInputs(HopperIOInputs inputs) {}


}
