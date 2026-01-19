package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX rollerMotor;
    private TalonFX deployerMotor;


    //control output types
    private VoltageOut rollerVoltageRequest;
    private TorqueCurrentFOC deployerCurrentRequest;
}
