package frc.robot.subsystems.fuelIntake;

import edu.wpi.first.math.filter.Debouncer;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final LoggedTunableNumber testingMode = 
        new LoggedTunableNumber("Intake/TestingMode", 0);
    private final LoggedTunableNumber rollerVelocityRPS = 
        new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0); 
    private final LoggedTunableNumber deployerPositionRotations = 
        new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);
    private final LoggedTunableNumber deployerVoltage = 
        new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);
    private final LoggedTunableNumber rollerVoltage = 
        new LoggedTunableNumber("Intake/RollerVoltage", 0.0);
    private final LoggedTunableNumber deployerCurrent = 
        new LoggedTunableNumber("Intake/DeployerCurrent", 0.0);

    
    private final Debouncer deployerDeployedDebouncer = new Debouncer(0.5);
    private final Debouncer deployerRetractedDebouncer = new Debouncer(0.5);

    

    public Intake(IntakeIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

    }

    

    public void intake() {
        io.setRollerVelocity(IntakeConstants.INTAKE_ROLLER_VELOCITY_RPS);
    }

    public void stopRoller(){
        io.setRollerVelocity(0.0);
    }

    public void deploy(){
        io.setDeployerPosition(IntakeConstants.DEPLOYED_POSITION_ROTATIONS);
    }

    public void retract(){
        io.setDeployerPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
    }

        public boolean isDeployed() {
           // Only consider deployed if the hardware is connected
           if (!inputs.deployerConnected) {
               return false;
           }else if (inputs.linearPosition.toRotations() >= IntakeConstants.DEPLOYED_POSITION_ROTATIONS) {
               return deployerDeployedDebouncer.calculate(true);
              } else {
               return deployerDeployedDebouncer.calculate(false);
              }
        }

        public boolean isRetracted() {
           // Only consider retracted if the hardware is connected
           if (!inputs.deployerConnected) {
               return false;
           }else if (inputs.linearPosition.toRotations() <= IntakeConstants.RETRACTED_POSITION_ROTATIONS) {
               return deployerRetractedDebouncer.calculate(true);
              } else {
               return deployerRetractedDebouncer.calculate(false);
              }
        }

}
