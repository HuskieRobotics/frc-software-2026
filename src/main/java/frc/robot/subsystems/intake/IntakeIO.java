package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // rollers
    double rollerVoltageLead = 0.0;
    double rollerVelocityRPSLead = 0.0;
    double rollerReferenceVelocityRPSLead = 0.0;
    double rollerStatorCurrentLead = 0.0;
    double rollerSupplyCurrentLead = 0.0;
    double rollerTempCelsiusLead = 0.0;
    double rollerClosedLoopErrorRPSLead = 0.0;
    double rollerClosedLoopReferenceRPSLead = 0.0;
    boolean rollerConnectedLead = false;

    // roller follower
    boolean rollerConnectedFollower = false;
    double rollerStatorCurrentFollower = 0.0;
    double rollerSupplyCurrentFollower = 0.0;
    double rollerVelocityRPSFollower = 0.0;
    double rollerTorqueCurrentFollower = 0.0;
    double rollerTempCelsiusFollower = 0.0;

    // deployer
    boolean deployerConnected = false;
    double deployerStatorCurrent = 0.0;
    double deployerSupplyCurrent = 0.0;
    double deployerVoltage = 0.0;
    double deployerTempCelsius = 0.0;

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
