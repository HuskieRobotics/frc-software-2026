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
    oi.getClimberUpButton()
        .whileTrue(Commands.run(() -> climber.setClimberVoltage(CLIMBER_EXTEND_VOLTAGE), climber))
        .onFalse(Commands.runOnce(() -> climber.stop(), climber));

    oi.getClimberDownButton()
        .whileTrue(Commands.run(() -> climber.setClimberVoltage(CLIMBER_RETRACT_VOLTAGE), climber));
  }

  public static Command getPrepareClimbCommand(Climber climber) {
    return Commands.runOnce(() -> climber.setClimberAngle(CLIMB_READY_ANGLE), climber)
        .withName("prepare climber");
  }

  public static Command getClimbAndHangCommand(Climber climber) {
    return Commands.runOnce(() -> climber.setClimberAngle(CLIMB_RETRACT_ANGLE), climber)
        .andThen(Commands.waitUntil(climber::isAngleAtSetpoint))
        .withName("climb and hang");
  }

  public static Command getStowClimberCommand(Climber climber) {
    return Commands.runOnce(() -> climber.setClimberAngle(Degrees.of(0.0)), climber)
        .andThen(Commands.waitUntil(climber::isAngleAtSetpoint))
        .withName("stow climber");
  }
}
