package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // rollers
    Voltage rollerVoltage = Volts.of(0);
    AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity rollerReferenceVelocityRadPerSec = RotationsPerSecond.of(0.0);
    Current rollerStatorCurrentAmps = Amps.of(0);
    Current rollerSupplyCurrentAmps = Amps.of(0);
    Temperature rollerTempCelsius = Celsius.of(0);
    Angle rollerClosedLoopError = Rotations.of(0);
    Angle rollerClosedLoopReference = Rotations.of(0);

    boolean rollerConnected = false;

    // deployer
    Voltage deployerVoltage = Volts.of(0);
    Current deployerStatorCurrentAmps = Amps.of(0);
    Current deployerSupplyCurrentAmps = Amps.of(0);
    Temperature deployerTempCelsius = Celsius.of(0);
    Angle deployerReferencePositionDeg = Rotations.of(0);
    Angle angularPosition = Rotations.of(0);
    AngularVelocity deployerClosedLoopError = RotationsPerSecond.of(0);
    AngularVelocity deployerClosedLoopReference = RotationsPerSecond.of(0);

    boolean deployerConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVelocity(AngularVelocity velocity) {}

  public default void setRollerVoltage(Voltage volts) {}

  public default void setRollerCurrent(Current amps) {}

  public default void setDeployerPosition(Angle position) {}

  public default void setDeployerVoltage(Voltage volts) {}
}
