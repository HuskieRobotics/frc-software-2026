package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommandsFactory {

  private ClimberCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Climber climber) {

    oi.getExecuteClimbButton().onTrue(getReadyToClimbCommand(climber));

    oi.getClimberUpButton()
        .whileTrue(Commands.run(() -> climber.setClimberVoltage(CLIMBER_EXTEND_VOLTAGE), climber));

    oi.getClimberDownButton()
        .whileTrue(Commands.run(() -> climber.setClimberVoltage(CLIMBER_RETRACT_VOLTAGE), climber));
  }

public static Command getReadyToClimbCommand(Climber climber) {
  return Commands.sequence(
      // keep the hook ready to climb if we aren't already there
      Commands.runOnce(() -> climber.setClimberAngle(CLIMB_ENGAGE_ANGLE), climber)
          .andThen(Commands.waitUntil(climber::isAngleAtSetpoint))
          .andThen(Commands.waitSeconds(0.25)),
      
      // pull the robot up so that we can climb onto the rung
      Commands.runOnce(() -> climber.setClimberAngle(CLIMB_RETRACT_ANGLE), climber)
          .andThen(Commands.waitUntil(climber::isAngleAtSetpoint))
          .andThen(Commands.waitSeconds(0.1)),
      
      // keep lifting up the robot if we need to 
      Commands.runOnce(() -> climber.setClimberAngle(MIN_ANGLE_DEGREES), climber)
          .andThen(Commands.waitUntil(climber::isAngleAtSetpoint))
          
  ).withName("climb sequence");
}

  public static Command prepareClimberCommand(Climber climber) {
    return Commands.runOnce(() -> climber.setClimberAngle(CLIMB_READY_ANGLE), climber)
        .withName("prepare climber");
  }
}
