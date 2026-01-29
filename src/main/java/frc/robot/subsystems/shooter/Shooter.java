package frc.robot.subsystems.shooter;


import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;


import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableBoolean;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;


import org.littletonrobotics.junction.Logger;


/**
* Example subsystem for controlling the velocity of an shooter mechanism with independently
* controlled top and bottom shooter wheels.
*
* <p>WARNING: This code is for example purposes only. It will not work with a physical shooter
* mechanism without changes. While it is derived from Huskie Robotics 2024 shooter, it has been
* simplified to highlight select best practices.
*
* <p>This example illustrates the following features:
*
* <ul>
*   <li>AdvantageKit support for logging and replay
*   <li>Use of an InterpolatingDoubleTreeMap to map distances to velocities
*   <li>Use of a debouncer to determine if the wheels are at a setpoint
*   <li>Use of logged tunable numbers for manual control and testing
*   <li>Use of a simulation class to model the wheels' behavior in simulation
*   <li>Use of a SysIdRoutine to perform system identification
*   <li>Use of a system check command to verify the shooter's functionality
*   <li>Use of a fault reporter to report issues with the shooter's devices
* </ul>
*/
public class Shooter extends SubsystemBase {
// all subsystems receive a reference to their IO implementation when constructed
private ShooterIO io;


 // all subsystems create the IO inputs instance for this subsystem
 private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();


 // When initially testing a mechanism, it is best to manually provide a voltage or current to
 // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
 // when multiple motors are involved, that is not possible. Using a tunables to enable testing
 // mode and, for the shooter, specifying current or velocity is convenient. This feature is also
 // an efficient approach when, for example, empirically tuning the velocity for different
 // distances when shooting a game piece.

 //testing mode creation of variables
 private final LoggedTunableNumber testingMode = 
     new LoggedTunableNumber("Shooter/TestingMode", 0);
 private final LoggedTunableNumber flyWheelLeadVelocity = 
     new LoggedTunableNumber("Shooter/FlyWheelLead Velocity", 0);
 private final LoggedTunableNumber flywheelLeadTorqueCurrent = 
     new LoggedTunableNumber("Shooter/FlyWheelLead TorqueCurrent", 0);
 private final LoggedTunableNumber turretPosition = 
     new LoggedTunableNumber("Shooter/Turret Position", 0);
 private final LoggedTunableNumber turretVoltage = 
     new LoggedTunableNumber("Shooter/Turret Voltage", 0);
  private final LoggedTunableNumber hoodPosition =
      new LoggedTunableNumber("Shooter/Hood Position", 0);
  private final LoggedTunableNumber hoodVoltage = 
      new LoggedTunableNumber("Shooter/Hood Voltage", 0);
  private final LoggedTunableBoolean isShooterReady =
     new LoggedTunableBoolean("Shooter/Is Shooter Ready", false);

 // The SysId routine is used to characterize the mechanism. While the SysId routine is intended to
 // be used for voltage control, we can apply a current instead and reinterpret the units when
 // performing the analysis in SysId.

private final SysIdRoutine flywheelIdRoutine =
     new SysIdRoutine(
         new SysIdRoutine.Config(
             Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
             Volts.of(10), // will actually be a step to 10 A
             Seconds.of(5), // override default timeout (10 s)
             // Log state with SignalLogger class
             state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
         new SysIdRoutine.Mechanism(
             output -> io.setFlywheelTorqueCurrent(Amps.of(output.in(Volts))),
             null,
             this)); // treat volts as amps

 public Shooter(ShooterIO io) {
   this.io = io;

   // Register this subsystem's SysId routine with the SysIdRoutineChooser. This allows
   // the routine to be selected and executed from the dashboard.
   SysIdRoutineChooser.getInstance()
       .addOption("Flywheel Lead Top Current", flywheelIdRoutine);


   // Register this subsystem's system check command with the fault reporter. The system check
   // command can be added to the Elastic Dashboard to execute the system test.
   FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
 }


 @Override
 public void periodic() {
   io.updateInputs(shooterInputs);
  Logger.processInputs("Shooter", shooterInputs);



   Logger.recordOutput("Shooter/IsShooterReady", this.isShooterReady.toString());
   Logger.recordOutput("Shooter/TestingMode", this.testingMode.toString());
   Logger.recordOutput("Shooter/HoodAngle", this.hoodPosition.toString());
   Logger.recordOutput("Shooter/TurretAngle", this.turretPosition.toString());
   if(isShooterReady.get()){
     Logger.recordOutput("Shooter/ShooterReady", true);
   } else {
     Logger.recordOutput("Shooter/ShooterReady", false);
   }
   
   //We might want to calculate the current target right here based off hood angle + turret angle

   if (testingMode.get() == 1) {

    //Flywheel Lead
    if(flyWheelLeadVelocity.get() != 0){
      io.setFlywheelLeadVelocity(RotationsPerSecond.of(flyWheelLeadVelocity.get()));
    }

    else if(flywheelLeadTorqueCurrent.get() != 0){
      io.setFlywheelTorqueCurrent(Amps.of(flywheelLeadTorqueCurrent.get()));
    }

    else if(turretPosition.get() != 0){
      io.setTurretPosition(Degrees.of(turretPosition.get()));
    }
    else if(turretVoltage.get() != 0){
      io.setTurretVoltage(Volts.of(turretVoltage.get()));
    }

    else if(hoodPosition.get() != 0){
      io.setHoodPosition(Degrees.of(hoodPosition.get()));
    }

    else if(hoodVoltage.get() != 0){
      io.setHoodVoltage(Volts.of(hoodVoltage.get()));
    }
  }



   if (testingMode.get() == 1) {
     io.setFlywheelLeadVelocity(ShooterIOInputs.flywheelLeadReferenceVelocity);
     io.setHoodPosition(ShooterIOInputs.hoodReferencePosition);
     io.setTurretPosition(ShooterIOInputs.turretReferencePosition);
   }
   //FIXME: Can also add more conditions based off velocities/positions

   // Log how long this subsystem takes to execute its periodic method.
   // This is useful for debugging performance issues.
   LoggedTracer.record("Shooter");
 }


 public boolean isShooterReady() {
   return (ShooterIO.flywheelLeadConnected &&
           ShooterIO.flywheelFollow1Connected &&
           ShooterIO.flywheelFollow2Connected &&
           ShooterIO.flywheelFollow3Connected &&
           ShooterIO.hoodConnected &&
           ShooterIO.turretConnected);
}

//LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE);


          

public void autoAim() {
  
}

// public void reverseShooter(double reverseAmps) { //FIXME: idk how to do this somebody do it
//  io.setFlywheelTorqueCurrent(-ShooterConstants.SHOOTER_CURRENT);
// }


/**
public double getPassingDistance(){
   // FIXME: Unsure if this is gonna be a command or a method
   return 0.0;
}
**/



// A subsystem's system check command is used to verify the functionality of the subsystem. It
// should perform a sequence of commands (usually encapsulated in another method). The command
// should always be decorated with an `until` condition that checks for faults in the subsystem
// and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
// faults are detected, the test will stop and the subsystem is always left in a safe state.
 private Command getSystemCheckCommand() { //FIXME: fix this once Kush lets us know

    Commands.runOnce(() -> io.set)}






 private Command getPresetCheckCommand(Distance distance) {
   return Commands.sequence(
       Commands.runOnce(() -> this.setVelocity(distance)),
       Commands.waitSeconds(2.0),
       Commands.runOnce(
           () ->
               this.checkVelocity(
                   RotationsPerSecond.of(shootingMap.get(distance.in(Meters))),
                   RotationsPerSecond.of(shootingMap.get(distance.in(Meters))))));
 }


 private void checkVelocity(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
   // check bottom motor
   if (!this.shooterInputs.shootMotorBottomVelocity.isNear(bottomVelocity, VELOCITY_TOLERANCE)) {
     FaultReporter.getInstance()
         .addFault(
             SUBSYSTEM_NAME,
             "Bottom shooter wheel velocity out of tolerance, should be "
                 + bottomVelocity
                 + " but is "
                 + this.shooterInputs.shootMotorBottomVelocity);
   }
   // check top motor
   if (!this.shooterInputs.shootMotorTopVelocity.isNear(topVelocity, VELOCITY_TOLERANCE)) {
     FaultReporter.getInstance()
         .addFault(
             SUBSYSTEM_NAME,
             "Top shooter wheel velocity out of tolerance, should be "
                 + topVelocity
                 + " but is "
                 + this.shooterInputs.shootMotorTopVelocity);
   }
 }



public void setFlywheelLeadVelocity(AngularVelocity velocity){      
   io.setFlywheelLeadVelocity(velocity);
 }


 public void setTurretPosition(Angle position){
   io.setTurretPosition(position);
 }


 public void setHoodPosition(Angle position){
   io.setHoodPosition(position);
 }


 public void setHoodVoltage(Voltage voltage){
   io.setHoodVoltage(voltage);
 }
}



