package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {

    // FLYWHEEL LEAD
    boolean flywheelLeadConnected = false;
    double flywheelLeadStatorCurrent = 0.0;
    double flywheelLeadSupplyCurrent = 0.0;
    double flywheelLeadVoltage = 0.0;
    double flywheelLeadTorqueCurrent = 0.0;
    double flywheelLeadVelocityRPS = 0.0;
    double flywheelLeadReferenceVelocityRPS = 0.0;
    double flywheelLeadClosedLoopReferenceVelocityRPS = 0.0;
    double flywheelLeadClosedLoopErrorVelocityRPS = 0.0;
    double flywheelLeadTemperature = 0.0;

    // FLYWHEEL FOLLOW 1
    boolean flywheelFollow1Connected = false;
    double flywheelFollow1StatorCurrent = 0.0;
    double flywheelFollow1SupplyCurrent = 0.0;
    double flywheelFollow1Voltage = 0.0;
    double flywheelFollow1TorqueCurrent = 0.0;
    double flywheelFollow1VelocityRPS = 0.0;
    double flywheelFollow1Temperature = 0.0;

    // FLYWHEEL FOLLOW 2
    boolean flywheelFollow2Connected = false;
    double flywheelFollow2StatorCurrent = 0.0;
    double flywheelFollow2SupplyCurrent = 0.0;
    double flywheelFollow2Voltage = 0.0;
    double flywheelFollow2TorqueCurrent = 0.0;
    double flywheelFollow2VelocityRPS = 0.0;
    double flywheelFollow2Temperature = 0.0;

    boolean turretConnected = false;
    double turretStatorCurrent = 0.0;
    double turretSupplyCurrent = 0.0;
    double turretVoltage = 0.0;
    double turretPositionRot = 0.0;
    double turretReferencePositionRot = 0.0;
    double turretClosedLoopReferencePositionRot = 0.0;
    double turretClosedLoopErrorPositionRot = 0.0;
    double turretTemperature = 0.0;
    double turretVelocityRPS = 0.0;

    boolean hoodConnected = false;
    double hoodStatorCurrent = 0.0;
    double hoodSupplyCurrent = 0.0;
    double hoodVoltage = 0.0;
    double hoodPositionRot = 0.0;
    double hoodReferencePositionRot = 0.0;
    double hoodClosedLoopReferencePositionRot = 0.0;
    double hoodClosedLoopErrorPositionRot = 0.0;
    double hoodTemperature = 0.0;

    // Fuel Detection
    boolean fuelDetectorConnected = false;
    boolean fuelDetectorHasFuel = false;
    double fuelDetectorDistanceToFuelMeters = 0.0; // FIXME: determine if necessary
    double fuelDetectorSignalStrength = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Flywheel IO methods
  public default void setFlywheelVelocity(double velocityRPS) {}

  public default void setFlywheelCurrent(double amps) {}

  // Turret IO methods
  public default void setTurretPosition(double positionRot) {}

  public default void setTurretVoltage(double voltage) {}

  // Hood IO methods
  public default void setHoodPosition(double positionRot) {}

  public default void setHoodVoltage(double voltage) {}

  public default void lowerHoodSlow(double voltage) {}

  public default void zeroTurretPosition() {}

  public default void zeroHoodPosition() {}
}
