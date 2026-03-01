package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommandsFactory {

  private IntakeCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Intake intake) {

    oi.getDeployRetractIntakeButton()
        .onTrue(
            Commands.either(
                    intake.getRetractAndStopCommand(),
                    intake.getDeployAndStartCommand(),
                    intake::inDeployedState)
                .withName("deploy-retract intake"));

    oi.getStopIntakeRollersButton()
        .onTrue(
            Commands.either(
                    Commands.runOnce(intake::stopRoller, intake),
                    Commands.runOnce(intake::startRoller, intake),
                    intake::areRollersActive)
                .withName("start-stop intake rollers"));
  }
}
