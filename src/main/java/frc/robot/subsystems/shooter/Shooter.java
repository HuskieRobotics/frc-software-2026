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
// we can fix this error once we create a instance of shooter input s whentesting

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
  private final LoggedTunableBoolean isShooterConnected =
     new LoggedTunableBoolean("Shooter/Is Shooter Connected", false);

 // The SysId routine is used to characterize the mechanism. While the SysId routine is intended to
 // be used for voltage control, we can apply a current instead and reinterpret the units when
 // performing the analysis in SysId.

private final SysIdRoutine flywheelIdRoutine =
     new SysIdRoutine(
         new SysIdRoutine.Config(
             Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
             Volts.of(10), // will actually be a step to 10 A
             Seconds.of(10), // override default timeout (10 s)
             // Log state with SignalLogger class
             state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
         new SysIdRoutine.Mechanism(
             output -> io.setFlywheelLeadTorqueCurrent(Amps.of(output.in(Volts))),
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
  FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getShooterSystemCheckCommand());
 }

 @Override
 public void periodic() {
  io.updateInputs(shooterInputs); // will be fixed once we build the code
   Logger.processInputs("Shooter", shooterInputs);
   Logger.recordOutput("Shooter/IsShooterConnected", this.isShooterConnected.toString());
   Logger.recordOutput("Shooter/TestingMode", this.testingMode.toString());
   Logger.recordOutput("Shooter/HoodAngle", this.hoodPosition.toString());
   Logger.recordOutput("Shooter/TurretAngle", this.turretPosition.toString());
   
   if(isShooterConnected.get()){
     Logger.recordOutput("Shooter/ShooterConnected", true);
   } else {
     Logger.recordOutput("Shooter/ShooterConnected", false);
   }
   
   //We might want to calculate the current target right here based off hood angle + turret angle

   if (testingMode.get() == 1) { //If we are testing
    
    // Set velocities/positions to reference (desired) values
     io.setFlywheelLeadVelocity(ShooterIO.flywheelLeadReferenceVelocity);
     io.setHoodPosition(ShooterIO.hoodReferencePosition);
     io.setTurretPosition(ShooterIO.turretReferencePosition);
     
    //Flywheel Lead
    if(flyWheelLeadVelocity.get() != 0){
      io.setFlywheelLeadVelocity(RotationsPerSecond.of(flyWheelLeadVelocity.get()));
    }

    else if(flywheelLeadTorqueCurrent.get() != 0){
      io.setFlywheelLeadTorqueCurrent(Amps.of(flywheelLeadTorqueCurrent.get()));
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

   //FIXME: Can also add more conditions based off velocities/positions

   // Log how long this subsystem takes to execute its periodic method.
   // This is useful for debugging performance issues.
   LoggedTracer.record("Shooter");
 }


 public boolean isShooterConnected() {
   return (ShooterIO.flywheelLeadConnected &&
           ShooterIO.flywheelFollow1Connected &&
           ShooterIO.flywheelFollow2Connected &&
           ShooterIO.hoodConnected &&
           ShooterIO.turretConnected);
}

//LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE); //FIXME: add leds later

 /**
  * Creates a command that performs a system check of the shooter subsystem.
  *
  * <p>The system check verifies that all motors and sensors are connected and functioning
  * properly.
  *
  * @return the system check command
  */
 public Command getShooterSystemCheckCommand() {
   return Commands.runOnce(
       () -> {
         boolean allConnected = isShooterConnected();
         if (!allConnected) {
           FaultReporter.getInstance()
               .addFault(
                   SUBSYSTEM_NAME,
                   "Shooter system check failed: One or more devices not connected.");
          
         }
         
       },
       this);
 }

 public void checkFlywheelVelocity(double flywheelLeadIntendedVelocityRPS, double flywheelFollow1IntendedVelocityRPS, double flywheelFollow2IntendedVelocityRPS) {
   //flywheel lead
  if(Math.abs(shooterInputs.flywheelLeadVelocity - flywheelLeadIntendedVelocityRPS) > VELOCITY_TOLERANCE)
   {
      if(Math.abs(shooterInputs.flywheelLeadVelocity) - Math.abs(flywheelLeadIntendedVelocityRPS) < 0)
      {
        FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel lead velocity is too low, should be " + flywheelLeadIntendedVelocityRPS + " but is " + shooterInputs.flywheelLeadVelocity);
      }
      else if(Math.abs(shooterInputs.flywheelLeadVelocity) - Math.abs(flywheelLeadIntendedVelocityRPS) > 0)
      {
        FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel lead velocity is too high, should be " + flywheelLeadIntendedVelocityRPS + " but is " + shooterInputs.flywheelLeadVelocity);
      }
   }

   //flywheel follow 1
   if(Math.abs(shooterInputs.flywheelFollow1Velocity) - Math.abs(flywheelFollow1IntendedVelocityRPS) > VELOCITY_TOLERANCE)
   {
      if(Math.abs(shooterInputs.flywheelFollow1Velocity) - Math.abs(flywheelFollow1IntendedVelocityRPS) < 0)
      {
        FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel follow 1 velocity is too low, should be " + flywheelFollow1IntendedVelocityRPS + " but is " + shooterInputs.flywheelFollow1Velocity);
      }
      else if(Math.abs(shooterInputs.flywheelFollow1Velocity) - Math.abs(flywheelFollow1IntendedVelocityRPS) > 0)
      {
        FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel follow 1 velocity is too high, should be " + flywheelFollow1IntendedVelocityRPS + " but is " + shooterInputs.flywheelFollow1Velocity);
      }
   }
    //flywheel follow 2
    if(Math.abs(shooterInputs.flywheelFollow2Velocity) - Math.abs(flywheelFollow2IntendedVelocityRPS) > VELOCITY_TOLERANCE)
    {
        if(Math.abs(shooterInputs.flywheelFollow2Velocity) - Math.abs(flywheelFollow2IntendedVelocityRPS) < 0)
        {
          FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel follow 2 velocity is too low, should be " + flywheelFollow2IntendedVelocityRPS + " but is " + shooterInputs.flywheelFollow2Velocity);
        }
        else if(Math.abs(shooterInputs.flywheelFollow2Velocity) - Math.abs(flywheelFollow2IntendedVelocityRPS) > 0)
        {
          FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "Flywheel follow 2 velocity is too high, should be " + flywheelFollow2IntendedVelocityRPS + " but is " + shooterInputs.flywheelFollow2Velocity);
        }
    }
  }
// public void reverseShooter(double reverseAmps) { //FIXME: unsure if this is needed
//  io.setFlywheelTorqueCurrent(-ShooterConstants.SHOOTER_CURRENT);
// }


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



