package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;

public class HopperCommandsFactory {

  private HopperCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Hopper hopper) {
    oi.getUnjamHopperButton().whileTrue(hopper.getUnjamCommand().withName("unjam hopper"));
    oi.getUnjamHopperButton().onFalse(Commands.runOnce(hopper::stop, hopper));
  }
}
