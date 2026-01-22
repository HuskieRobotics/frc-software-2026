package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class ShooterIOInputs {

    // Kicker inputs
    boolean kickerConnected = false;
    Current kickerStatorCurret = Amps.of(0.0);
    Current kickerSupplyCurret = Amps.of(0.0);
    Voltage kickerVoltage = Volts.of(0.0);
    Temperature kickTemperature = Celsius.of(0.0);

    // Flywheel inputs
    boolean flywheelConnected = false;
    Current flywheelStatorCurrent = Amps.of(0.0);
    Current flywheelSupplyCurrent = Amps.of(0.0);
    Current flywheelTorqueCurrent = Amps.of(0.0);
    AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelReferenceVelocity = RotationsPerSecond.of(0.0); // chached from set method
    AngularVelocity flywheelClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0); // only logged when TUNING is set
    AngularVelocity flywheelClosedLoopErrorVelocity = RotationsPerSecond.of(0.0); // only logged when TUNING is set

    // Turret inputs
    boolean turretConnected = false;
    Current turretStatorCurrent = Amps.of(0.0);
    Current turretSupplyCurrent = Amps.of(0.0);
    
    Voltage turretVoltage = Volts.of(0.0);
    Angle turretPosition = Degrees.of(0.0);
    Angle turretReferencePosition = Degrees.of(0.0); // cached from set method
    Angle turretClosedLoopReferencePosition = Degrees.of(0.0); // only logged when TUNING is set
    Angle turretClosedLoopErrorPosition = Degrees.of(0.0); // only logged when TUNING is set

    // Hood inputs
    boolean hoodConnected = false;
    Current hoodStatorCurrent = Amps.of(0.0);
    Current hoodSupplyCurrent = Amps.of(0.0);
    Voltage hoodVoltage = Volts.of(0.0);
    Angle hoodPosition = Degrees.of(0.0);
    Angle hoodReferencePosition = Degrees.of(0.0); // cached from set method
    Angle hoodClosedLoopReferencePosition = Degrees.of(0.0); // only logged when TUNING is set
    Angle hoodClosedLoopErrorPosition = Degrees.of(0.0); // only logged when TUNING is set
    
    // Encoder inputs (tbd)

  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterWheelTopVelocity(AngularVelocity velocity) {}

  public default void setShooterWheelBottomVelocity(AngularVelocity velocity) {}

  public default void setShooterWheelTopCurrent(Current amps) {}

  public default void setShooterWheelBottomCurrent(Current amps) {}

  // Kicker IO methods
  public default void setKickerMotorIntakeVoltage(Voltage voltage) {}

  // Flywheel IO methods
  public default void setFlywheelVelocity(AngularVelocity velocity) {}

  public default void setFlywheelTorqueCurrent(Current amps) {}

  // Turret IO methods
  public default void setTurretPosition(Angle position) {}

  public default void setTurretVoltage(Voltage voltage) {}

  // Hood IO methods
  public default void setHoodPosition(Angle position) {}

  public default void setHoodVoltage(Voltage voltage) {}
}
