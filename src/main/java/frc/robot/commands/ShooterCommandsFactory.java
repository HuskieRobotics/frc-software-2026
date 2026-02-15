package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommandsFactory {

  private ShooterCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Shooter shooter) {

    oi.getZeroHoodButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(shooter::lowerHoodSlow, shooter),
                    Commands.waitUntil(shooter::isHoodJammed),
                    Commands.runOnce(shooter::stopHood, shooter),
                    Commands.runOnce(shooter::zeroHood, shooter))
                .withName("zero hood"));
  }
}
