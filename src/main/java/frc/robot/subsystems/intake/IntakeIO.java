package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // rollers
    double rollerVoltage = 0.0;
    double rollerVelocityRPS = 0.0;
    double rollerReferenceVelocityRPS = 0.0;
    double rollerStatorCurrent = 0.0;
    double rollerSupplyCurrent = 0.0;
    double rollerTempCelsius = 0.0;
    double rollerClosedLoopErrorRPS = 0.0;
    double rollerClosedLoopReferenceRPS = 0.0;
    boolean rollerConnected = false;

    // deployer lead
    boolean deployerConnectedLead = false;
    double deployerStatorCurrentLead = 0.0;
    double deployerSupplyCurrentLead = 0.0;
    double deployerVoltageLead = 0.0;
    double deployerTempCelsiusLead = 0.0;

    // deployer follower
    boolean deployerConnectedFollower = false;
    double deployerStatorCurrentFollower = 0.0;
    double deployerSupplyCurrentFollower = 0.0;
    double deployerVoltageFollower = 0.0;
    double deployerTempCelsiusFollower = 0.0;

    double deployerReferencePositionRot = 0.0;
    double deployerAngularPositionRot = 0.0;

    double deployerClosedLoopErrorRot = 0.0;
    double deployerClosedLoopReferenceRot = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVelocity(double velocityRPS) {}

  public default void setRollerCurrent(double amps) {}

  public default void setDeployerPosition(double positionRot) {}

  public default void setDeployerVoltage(double volts) {}

  public default void setDeployerCurrent(double amps) {}
}
