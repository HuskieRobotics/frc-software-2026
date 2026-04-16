package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {

    // Connections
    boolean kickerMotorConnected = false;
    boolean spindexerMotorConnected = false;

    // Voltage
    double kickerVoltageSupplied = 0.0;
    double spindexerVoltageSupplied = 0.0;

    // Temp
    double kickerTemperatureCelsius = 0.0;
    double spindexerTemperatureCelsius = 0.0;

    // Stator
    double spindexerStatorCurrent = 0.0;
    double kickerStatorCurrent = 0.0;

    // Supply
    double spindexerSupplyCurrent = 0.0;
    double kickerSupplyCurrent = 0.0;

    // Angular Velocity
    double spindexerVelocityRPS = 0.0;
    double kickerVelocityRPS = 0.0;

    double spindexerReferenceVelocityRPS = 0.0;
    double kickerReferenceVelocityRPS = 0.0;

    double closedLoopErrorSpindexerRPS = 0.0;
    double closedLoopErrorKickerRPS = 0.0;
    double closedLoopReferenceVelocitySpindexerRPS = 0.0;
    double closedLoopReferenceVelocityKickerRPS = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setSpindexerVelocity(double velocityRPS) {}

  public default void setKickerVelocity(double velocityRPS) {}

  public default void setSpindexerCurrent(double amps) {}

  public default void setKickerCurrent(double amps) {}
}
