package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // rollers
    Voltage rollerVoltage = Volts.of(0);
    AngularVelocity rollerVelocityRPS = RotationsPerSecond.of(0.0);
    Current rollerStatorCurrentAmps = Amps.of(0);
    Current rollerSupplyCurrentAmps = Amps.of(0);
    Temperature rollerTempCelcius = Celsius.of(0);
    AngularVelocity rollerReferenceVelocityRadPerSec = RotationsPerSecond.of(0);
    Angle rollerClosedLoopError = Rotations.of(0);
    Angle rollerClosedLoopReference = Rotations.of(0);

    boolean rollerConnected = false;

    // deployer
    Voltage deployerVoltage = Volts.of(0);
    Current deployerStatorCurrentAmps = Amps.of(0);
    Current deployerSupplyCurrentAmps = Amps.of(0);
    Temperature deployerTempCelcius = Celsius.of(0);
    Angle deployerReferencePositionDeg = Rotations.of(0);
    Distance linearPosition = Meters.of(0);
    Angle angularPosition = Rotations.of(0);
    Angle deployerClosedLoopError = Rotations.of(0);
    Angle deployerClosedLoopReference = Rotations.of(0);

    boolean deployerConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVelocity(double velocity) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void setRollerCurrent(Current Amps) {}

  public default void setDeployerPosition(Distance linearDistance) {}

  public default void setDeployerVoltage(Voltage Volts) {}

  public default void setDeployerCurrent(Current Amps) {}
}
