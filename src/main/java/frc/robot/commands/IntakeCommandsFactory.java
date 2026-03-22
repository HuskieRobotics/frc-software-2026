package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommandsFactory {

  private IntakeCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Intake intake) {

    oi.getForceDeployIntakeButton()
        .onTrue(intake.getDeployAndStartCommand().withName("force-deploy intake"));

    oi.getDeployRetractIntakeButton()
        .onTrue(
            Commands.either(
                    intake.getRetractAndStopCommand(),
                    intake.getDeployAndStartCommand(),
                    intake::inDeployedState)
                .withName("deploy-retract intake"));

    oi.getStartStopIntakeRollersButton()
        .onTrue(
            Commands.either(
                    Commands.runOnce(intake::stopRoller, intake),
                    Commands.runOnce(intake::startRoller, intake),
                    intake::areRollersActive)
                .withName("start-stop intake rollers"));

    oi.getReverseIntakeRollersButton()
        .and(intake::isDeployed)
        .whileTrue(Commands.runOnce(intake::reverseRoller, intake));

    oi.getReverseIntakeRollersButton().onFalse(Commands.runOnce(intake::startRoller, intake));

    FaultReporter.getInstance()
        .registerSystemCheck(
            IntakeConstants.SUBSYSTEM_NAME,
            intake.getSystemCheckCommand(),
            oi.getIntakeSystemTest());
  }
}
