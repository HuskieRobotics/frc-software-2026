package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import frc.robot.subsystems.fuelIntake.IntakeConstants;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team6328.util.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX rollerMotor;
    private TalonFX deployerMotor;


    //control output types
    private VoltageOut rollerVoltageRequest;
    private TorqueCurrentFOC deployerCurrentRequest;

    private Alert configAlert = 
        new Alert("Failed to apply configuration for Intake.", AlertType.kError);

    private final LoggedTunableNumber deployerKp =
        new LoggedTunableNumber("Intake/Deployer/kP", IntakeConstants.DEPLOYER_KP);
    private final LoggedTunableNumber deployerKi =
        new LoggedTunableNumber("Intake/Deployer/kI", IntakeConstants.DEPLOYER_KI);
    private final LoggedTunableNumber deployerKd =
        new LoggedTunableNumber("Intake/Deployer/kD", IntakeConstants.DEPLOYER_KD);
    private final LoggedTunableNumber deployerKs =
        new LoggedTunableNumber("Intake/Deployer/kS", IntakeConstants.DEPLOYER_KS);
    private final LoggedTunableNumber deployerKv =
        new LoggedTunableNumber("Intake/Deployer/kV", IntakeConstants.DEPLOYER_KV);
    private final LoggedTunableNumber deployerKa =
        new LoggedTunableNumber("Intake/Deployer/kA", IntakeConstants.DEPLOYER_KP);
}
