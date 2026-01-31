package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableBoolean;
import frc.lib.team6328.util.LoggedTunableNumber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.SUBSYSTEM_NAME;
import static frc.robot.subsystems.hopper.HopperConstants.ROLLER_CURRENT_SPIKE_THRESHHOLD_AMPS;
import static frc.robot.subsystems.hopper.HopperConstants.ROLLER_CURRENT_SPIKE_THRESHOLD_SECONDS;
import static frc.robot.subsystems.hopper.HopperConstants.ROLLER_MOTOR_MANUAL_CONTROL_VOLTAGE;
import static frc.robot.subsystems.hopper.HopperConstants.SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS;
import static frc.robot.subsystems.hopper.HopperConstants.SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS;
import static frc.robot.subsystems.hopper.HopperConstants.SPINDEXER_MOTOR_MANUAL_CONTROL_VOLTAGE;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperConstants.*;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged(); 
    
    private final LoggedTunableNumber spindexerVelocity =  new LoggedTunableNumber("Hopper/SpindexerVelocity", 0);
    private final LoggedTunableNumber rollerVelocity = new LoggedTunableNumber("Hopper/RollerVelocity", 0);
    private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Hopper/TestingMode", 0);

    private CurrentSpikeDetector spindexerSpikeDetector = new CurrentSpikeDetector(SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS, SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS);
    private CurrentSpikeDetector rollerSpikeDetector = new CurrentSpikeDetector(ROLLER_CURRENT_SPIKE_THRESHHOLD_AMPS, ROLLER_CURRENT_SPIKE_THRESHOLD_SECONDS);

    //private Alert spindexerJammedAlert = new Alert("Spindexer jam detected.", AlertType.kError);
    //private Alert rollerJammedAlert = new Alert("Kicker jam detected. This is the motor that kicks fuel from hopper into shooter.", AlertType.kError);
    
    public Hopper(HopperIO io){
        this.io = io;

        FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getHopperSystemCheckCommand());
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs(SUBSYSTEM_NAME, inputs);
        
        if (testingMode.get()==1){
            if(spindexerVelocity.get()!=0){
                setSpindexerVelocity(spindexerVelocity.get());
            }
            if(rollerVelocity.get()!=0) {
                setRollerVelocity(rollerVelocity.get());
            }
        }

        LoggedTracer.record(SUBSYSTEM_NAME);
    }

    private Command getHopperSystemCheckCommand() { // FIXME: This entire method may need to be updated. We are not sure of it.
        return Commands.sequence(
            Commands.runOnce(()->io.setRollerVoltage(ROLLER_MOTOR_MANUAL_CONTROL_VOLTAGE)), // FIXME
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> { 
                    if(inputs.RollerVelocity.lt(rollerVelocity)) {
                        FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME,"not moving as fast",false);

                    }
                }),
            Commands.runOnce(()->io.setSpindexerVoltage(SPINDEXER_MOTOR_MANUAL_CONTROL_VOLTAGE)), // FIXME
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> { 
                    if(inputs.spindexerVelocity.lt(spindexerVelocity)) {
                        FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME,"not moving as fast",false);

                    }
                })
                .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
                .andThen(Commands.runOnce(() -> io.setRollerVoltage(0.0))));
            
            
    }

    public void setRollerVelocity(double velocity) {
        io.setRollerVelocity(Units.RotationsPerSecond.of(velocity));
    }
    
    public void setSpindexerVelocity(double velocity) {
         io.setSpindexerVelocity(Units.RotationsPerSecond.of(velocity));
    }

    public double getRollerVelocity() {
        return rollerVelocity.get();
    }

    public double getSpindexerVelocity() {
        return spindexerVelocity.get();
    }
    
}


