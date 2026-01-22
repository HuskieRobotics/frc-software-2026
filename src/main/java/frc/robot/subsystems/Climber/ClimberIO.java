package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    // logging values
    double voltageSupplied = 0.0;
    double statorCurrentAmps = 0.0;
    double supplyCurrentAmps = 0.0;
    double tempCelsius = 0.0;

    boolean cageCatcherLimitSwitchEngaged = false;

    Angle closedLoopError = Rotations.of(0.0);
    Angle closedLoopReference = Rotations.of(0.0);

    Distance linearPosition = Meters.of(0.0);
    Angle angularPosition = Rotations.of(0.0);
  }

  public default void setPosition(Distance position) {}

  public default void setVoltage(double voltage) {}

  public default void zeroPosition() {}
}
