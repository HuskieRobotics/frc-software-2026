package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableBoolean;
import frc.lib.team6328.util.LoggedTunableNumber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import org.littletonrobotics.junction.Logger; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperConstants.*;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged(); 
    
    private final LoggedTunableNumber spindexerVelocity =  new LoggedTunableNumber("Hopper/SpindexerVelocity", 0);
    private final LoggedTunableNumber rollerVelocity = new LoggedTunableNumber("Hopper/KickerVelocity", 0);
    private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Hopper/TestingMode", 0);

    private CurrentSpikeDetector spindexerSpikeDetector = new CurrentSpikeDetector(SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS, SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS);
    private CurrentSpikeDetector rollerSpikeDetector = new CurrentSpikeDetector(ROLLER_CURRENT_SPIKE_THRESHHOLD_AMPS, ROLLER_CURRENT_SPIKE_THRESHOLD_SECONDS);

    private Alert spindexerJammedAlert = new Alert("Spindexer jam detected.", AlertType.kError);
    private Alert rollerJammedAlert = new Alert("Kicker jam detected. This is the motor that kicks fuel from hopper into shooter.", AlertType.kError);
    
    public Hopper(HopperIO io){
        this.io = io;

        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getHopperSystemCheckCommand());
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs(SUBSYSTEM_NAME, inputs);

        if (spindexerSpikeDetector.update(Math.abs(inputs.spindexerStatorCurrent.in(Amps)))) {
            CommandScheduler.getInstance()
                .schedule(
                    Commands.sequence(
                        Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_UNJAM_VELOCITY)).withName("spindexer jammed")
                    )
                );
            spindexerJammedAlert.set(true);
        } else {
            spindexerJammedAlert.set(false);
        }

        if (rollerSpikeDetector.update(Math.abs(inputs.spindexerStatorCurrent.in(Amps)))) {
            CommandScheduler.getInstance()
                .schedule(
                    Commands.sequence(
                        Commands.runOnce(() -> io.setRollerVelocity(ROLLER_UNJAM_VELOCITY)).withName("kicker jammed")
                    )
                );
            rollerJammedAlert.set(true);
        } else {
            rollerJammedAlert.set(false);
        }
        
        if (testingMode.get()==1){
            if(spindexerVelocity.get()!=0){
                setSpindexerVelocity(RotationsPerSecond.of(spindexerVelocity.get()));
            }
            if(rollerVelocity.get()!=0) {
                setRollerVelocity(RotationsPerSecond.of(rollerVelocity.get()));
            }
        }

        LoggedTracer.record(SUBSYSTEM_NAME);
    }

    private Command getHopperSystemCheckCommand() { // FIXME: This entire method may need to be updated. We are not sure of it.
        return Commands.sequence(
            Commands.runOnce(()->io.setRollerVelocity(RotationsPerSecond.of(ROLLER_MOTOR_MANUAL_CONTROL_VELOCITY))), // FIXME
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> { 
                    if(inputs.RollerVelocity.lt(rollerVelocity)) {
                        FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME,"kicker not moving as fast",false);

                    }
                }),
            Commands.runOnce(()->io.setSpindexerVelocity(RotationsPerSecond.of(SPINDEXER_MOTOR_MANUAL_CONTROL_VELOCITY))),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> { 
                    if(inputs.spindexerVelocity.lt(spindexerVelocity)) {
                        FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME,"spindexer not moving as fast",false);

                    }
                })
                .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
                .andThen(Commands.runOnce(() -> io.setRollerVelocity(RotationsPerSecond.of(0.0)))));
            
            
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        io.setRollerVelocity(velocity);
    }
    
    public void setSpindexerVelocity(AngularVelocity velocity) {
         io.setSpindexerVelocity(velocity);
    }

    public double getRollerVelocity() {
        return rollerVelocity.get();
    }

    public double getSpindexerVelocity() {
        return spindexerVelocity.get();
    }
    
}


