package frc.robot.subsystems.intake;

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
    AngularVelocity rollerReferenceVelocity = RotationsPerSecond.of(0.0);
    Current rollerStatorCurrent = Amps.of(0);
    Current rollerSupplyCurrent = Amps.of(0);
    Temperature rollerTempCelsius = Celsius.of(0);
    Angle rollerClosedLoopError = Rotations.of(0);
    Angle rollerClosedLoopReference = Rotations.of(0);

    boolean rollerConnected = false;

    // deployer
    boolean deployerConnected = false;
    Current deployerStatorCurrent = Amps.of(0);
    Current deployerSupplyCurrent = Amps.of(0);
    Voltage deployerVoltage = Volts.of(0);
    Temperature deployerTempCelsius = Celsius.of(0);
    Angle deployerReferencePosition = Rotations.of(0);
    Angle deployerAngularPosition = Rotations.of(0);

    Angle deployerClosedLoopError = Rotations.of(0);
    Angle deployerClosedLoopReference = Rotations.of(0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVelocity(AngularVelocity velocity) {}

  public default void setRollerCurrent(Current amps) {}

  public default void setDeployerPosition(Angle position) {}

  public default void setDeployerVoltage(Voltage volts) {}
}
