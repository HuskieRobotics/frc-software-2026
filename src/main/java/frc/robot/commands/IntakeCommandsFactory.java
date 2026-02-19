package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommandsFactory {

  private IntakeCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Intake intake) {

    oi.getDeployIntakeButton()
        .onTrue(
            Commands.either(
                Commands.parallel(
                        Commands.runOnce(intake::deployIntake, intake),
                        Commands.runOnce(intake::startRoller, intake))
                    .withName("Deploy Intake and Start Rollers"),
                Commands.parallel(
                        Commands.runOnce(intake::retractIntake, intake),
                        Commands.runOnce(intake::stopRoller, intake))
                    .withName("Retract Intake and Stop Rollers"),
                () -> !intake.isIntakeDeployed));
  }
}
