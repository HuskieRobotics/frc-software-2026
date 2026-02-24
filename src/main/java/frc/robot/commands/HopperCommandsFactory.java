package frc.robot.commands;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;

public class HopperCommandsFactory {

  private HopperCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Hopper hopper) {
    oi.getUnjamHopperButton()
        .onTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.runOnce(() -> hopper.setKickerVelocity(KICKER_UNJAM_VELOCITY))
                        .withName("kicker jammed"),
                    Commands.waitSeconds(KICKER_UNJAM_WAIT_TIME)),
                Commands.sequence(
                    Commands.runOnce(() -> hopper.setSpindexerVelocity(SPINDEXER_UNJAM_VELOCITY))
                        .withName("spindexer jammed"),
                    Commands.waitSeconds(SPINDEXER_UNJAM_WAIT_TIME))));
  }
}
