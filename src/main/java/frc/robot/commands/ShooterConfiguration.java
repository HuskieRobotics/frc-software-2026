package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team6328.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterConfiguration extends Command {

    private final SwerveDrivetrain drivetrain;
    private final Shooter shooter;

    private final boolean isHubActive;

    private final boolean passToggled;

    private String gameData;


    public enum ShooterMode {
        SHOOT,
        SHOOT_OTM,
        COLLECT,
        PASS
    }

    public ShooterConfiguration(SwerveDrivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;



    }

    @Override
    public void initialize() {
        // Configure shooter settings here
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
    }

 
    public ShooterMode getMode(){
        if (inAllianceZone()) {
            if(hubActive()){
                return ShooterMode.SHOOT;
            } else {
                return ShooterMode.COLLECT;
            }
        }
        else{
            
            if(passToggled){ // pass toggeled by Operator
                return ShooterMode.PASS;
            } else {
                return ShooterMode.SHOOT_OTM;
                
            }

            }
        }

    @Override
    public void getTrajectory(){


        if(getMode() == ShooterMode.SHOOT || getMode() == ShooterMode.COLLECT){} // model for aimed position
        else if(getMode() == ShooterMode.SHOOT_OTM){} // model for OTM pos
        else if(getMode() == ShooterMode.PASS){} // model for aimed position, which would be the nearest position
    }

    @Override
    public void getTurret(){

        if(getMode() == ShooterMode.SHOOT || getMode() == ShooterMode.COLLECT){} // aim turret at hub
        else if(getMode() == ShooterMode.SHOOT_OTM){} // aim the turret at transforme dhub based on velocity
        else if(getMode() == ShooterMode.PASS){} // aim turret at nearest corner
    }


    // make region2d with mechanical advantage field constants
    public boolean inAllianceZone() {
        return false;
    }

    // based on match time (which should be equivalent to the timer of this command as it is enabled)
    // and game data to see which hub was active first
    // add a t second offset for how long it takes the ball (on average) to actually enter the hub
    public boolean hubActive() {
        return true;
    }
}
