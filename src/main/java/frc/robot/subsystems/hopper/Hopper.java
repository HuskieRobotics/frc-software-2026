package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import edu.wpi.first.units.Units;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged(); 
    
    private final LoggedTunableNumber spindexerVelocity =  new LoggedTunableNumber("Hopper/SpindexerVelocity", 0);
    private final LoggedTunableNumber rollerVelocity = new LoggedTunableNumber("Hopper/RollerVelocity", 0);
    
    public Hopper(HopperIO io){
        this.io = io;

    }

    public void setRollerVelocity(double velocity) {
        io.setRollerVelocity(Units.RotationsPerSecond.of(velocity));
    }
    
    public void setSpindexerVelocity(double velocity) {
         io.setSpindexerVelocity(Units.RotationsPerSecond.of(velocity));
    }

}