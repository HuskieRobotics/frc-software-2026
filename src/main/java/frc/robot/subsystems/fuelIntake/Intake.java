package frc.robot.subsystems.fuelIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;

    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();  

    

    public Intake(IntakeIO io) {
        this.io = io;
    }
    
}
