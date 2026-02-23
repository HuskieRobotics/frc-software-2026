package frc.robot.commands;

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
                    Commands.sequence(
                        Commands.runOnce(intake::stopRoller),
                        Commands.runOnce(intake::retractIntake),
                        Commands.waitUntil(intake::isRetracted)),
                    Commands.sequence(
                        Commands.runOnce(intake::deployIntake),
                        Commands.waitUntil(intake::isDeployed),
                        Commands.runOnce(intake::startRoller)),
                    intake::inDeployedState)
                .withName("deploy-retract intake"));

    oi.getStopIntakeRollersButton()
        .onTrue(
            Commands.either(
                    Commands.runOnce(intake::stopRoller),
                    Commands.runOnce(intake::startRoller),
                    intake::areRollersActive)
                .withName("start-stop intake rollers"));
  }
}
