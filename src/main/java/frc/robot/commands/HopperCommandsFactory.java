package frc.robot.commands;

import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;

public class HopperCommandsFactory {

  private HopperCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Hopper hopper) {
    oi.getUnjamHopperButton().onTrue(hopper.getUnjamCommand().withName("unjam hopper"));
  }
}
