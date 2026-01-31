package frc.robot.subsystems.shooter;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.

  @AutoLog
  // IO Inputs are below
  public static class ShooterIOInputs {

    // FLYWHEEL LEAD
    boolean flywheelLeadConnected = false;
    Current flywheelLeadStatorCurrent = Amps.of(0.0);
    Current flywheelLeadSupplyCurrent = Amps.of(0.0);
    Current flywheelLeadTorqueCurrent = Amps.of(0.0);
    Voltage flywheelLeadVoltage = Volts.of(0.0);
    AngularVelocity flywheelLeadVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature flywheelLeadTemperature = Celsius.of(0.0);

    // FLYWHEEL FOLLOW 1
    boolean flywheelFollow1Connected = false;
    Current flywheelFollow1StatorCurrent = Amps.of(0.0);
    Current flywheelFollow1SupplyCurrent = Amps.of(0.0);
    Current flywheelFollow1TorqueCurrent = Amps.of(0.0);
    Voltage flywheelFollow1Voltage = Volts.of(0.0);
    AngularVelocity flywheelFollow1Velocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow1ReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow1ClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow1ClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature flywheelFollow1Temperature = Celsius.of(0.0);

    // FLYWHEEL FOLLOW 2
    boolean flywheelFollow2Connected = false;
    Current flywheelFollow2StatorCurrent = Amps.of(0.0);
    Current flywheelFollow2SupplyCurrent = Amps.of(0.0);
    Current flywheelFollow2TorqueCurrent = Amps.of(0.0);
    Voltage flywheelFollow2Voltage = Volts.of(0.0);
    AngularVelocity flywheelFollow2Velocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow2ReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow2ClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelFollow2ClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature flywheelFollow2Temperature = Celsius.of(0.0);

    boolean turretConnected = false;
    Current turretStatorCurrent = Amps.of(0.0);
    Current turretSupplyCurrent = Amps.of(0.0);
    Voltage turretVoltage = Volts.of(0.0);
    Angle turretPosition = Degrees.of(0.0);
    Angle turretReferencePosition = Degrees.of(0.0);
    Angle turretClosedLoopReferencePosition = Degrees.of(0.0);
    Angle turretClosedLoopErrorPosition = Degrees.of(0.0);
    Temperature turretTemperature = Celsius.of(0.0);

    boolean hoodConnected = false;
    Current hoodStatorCurrent = Amps.of(0.0);
    Current hoodSupplyCurrent = Amps.of(0.0);
    Voltage hoodVoltage = Volts.of(0.0);
    Angle hoodPosition = Degrees.of(0.0);
    Angle hoodReferencePosition = Degrees.of(0.0);
    Angle hoodClosedLoopReferencePosition = Degrees.of(0.0);
    Angle hoodClosedLoopErrorPosition = Degrees.of(0.0);
    Temperature hoodTemperature = Celsius.of(0.0);
    
    Angle encoderAngleDegrees = Degrees.of(0.0);
    AngularVelocity encoderAngularVelocityRPS = RotationsPerSecond.of(0.0);
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Flywheel IO methods
  public default void setFlywheelLeadVelocity(AngularVelocity velocity) {}
  public default void setFlywheelLeadTorqueCurrent(Current amps) {}
  
  // Turret IO methods
  public default void setTurretPosition(Angle position) {}
  public default void setTurretVoltage(Voltage voltage) {} 

  // Hood IO methods
  public default void setHoodPosition(Angle position) {}
  public default void setHoodVoltage(Voltage voltage) {}
}
