package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {

    // FLYWHEEL LEAD
    boolean flywheelLeadConnected = false;
    Current flywheelLeadStatorCurrent = Amps.of(0.0);
    Current flywheelLeadSupplyCurrent = Amps.of(0.0);
    Voltage flywheelLeadVoltage = Volts.of(0.0);
    Current flywheelLeadTorqueCurrent = Amps.of(0.0);
    AngularVelocity flywheelLeadVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity flywheelLeadClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature flywheelLeadTemperature = Celsius.of(0.0);

    // FLYWHEEL FOLLOW 1
    boolean flywheelFollow1Connected = false;
    Current flywheelFollow1StatorCurrent = Amps.of(0.0);
    Current flywheelFollow1SupplyCurrent = Amps.of(0.0);
    Voltage flywheelFollow1Voltage = Volts.of(0.0);
    Current flywheelFollow1TorqueCurrent = Amps.of(0.0);
    AngularVelocity flywheelFollow1Velocity = RotationsPerSecond.of(0.0);
    Temperature flywheelFollow1Temperature = Celsius.of(0.0);

    // FLYWHEEL FOLLOW 2
    boolean flywheelFollow2Connected = false;
    Current flywheelFollow2StatorCurrent = Amps.of(0.0);
    Current flywheelFollow2SupplyCurrent = Amps.of(0.0);
    Voltage flywheelFollow2Voltage = Volts.of(0.0);
    Current flywheelFollow2TorqueCurrent = Amps.of(0.0);
    AngularVelocity flywheelFollow2Velocity = RotationsPerSecond.of(0.0);
    Temperature flywheelFollow2Temperature = Celsius.of(0.0);

    boolean turretConnected = false;
    Current turretStatorCurrent = Amps.of(0.0);
    Current turretSupplyCurrent = Amps.of(0.0);
    Voltage turretVoltage = Volts.of(0.0);
    Angle turretPosition = Rotations.of(0.0);
    Angle turretReferencePosition = Rotations.of(0.0);
    Angle turretClosedLoopReferencePosition = Rotations.of(0.0);
    Angle turretClosedLoopErrorPosition = Rotations.of(0.0);
    Temperature turretTemperature = Celsius.of(0.0);

    boolean hoodConnected = false;
    Current hoodStatorCurrent = Amps.of(0.0);
    Current hoodSupplyCurrent = Amps.of(0.0);
    Voltage hoodVoltage = Volts.of(0.0);
    Angle hoodPosition = Rotations.of(0.0);
    Angle hoodReferencePosition = Rotations.of(0.0);
    Angle hoodClosedLoopReferencePosition = Rotations.of(0.0);
    Angle hoodClosedLoopErrorPosition = Rotations.of(0.0);
    Temperature hoodTemperature = Celsius.of(0.0);

    // Fuel Detection
    boolean fuelDetectorConnected = false;
    boolean fuelDetectorHasFuel = false;
    boolean isKickerJammed = false;
    Distance fuelDetectorDistanceToFuel = Meters.of(0.0); // FIXME: determine if necessary
    double fuelDetectorSignalStrength = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Flywheel IO methods
  public default void setFlywheelVelocity(AngularVelocity velocity) {}

  public default void setFlywheelCurrent(Current amps) {}

  // Turret IO methods
  public default void setTurretPosition(Angle position) {}

  public default void setTurretVoltage(Voltage voltage) {}

  // Hood IO methods
  public default void setHoodPosition(Angle position) {}

  public default void setHoodVoltage(Voltage voltage) {}

  public default void zeroTurretPosition() {}

  public default void zeroHoodPosition() {}

  public default void isKickerJammed() {}

  public default void hasFuel() {}

  public default void getDistanceToFuel() {}
}
