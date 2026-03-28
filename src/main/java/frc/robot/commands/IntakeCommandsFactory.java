package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
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
        .onTrue(
            Commands.parallel(
                    Commands.runOnce(intake::reverseRoller, intake),
                    Commands.run(
                        () -> LEDs.getInstance().requestState(LEDs.States.REVERSE_ROLLERS)))
                .withName("reverse intake rollers"));

    oi.getReverseIntakeRollersButton()
        .onFalse(Commands.runOnce(intake::startRoller, intake).withName("start intake rollers"));

    oi.getIncrementRollerVelocityButton().onTrue(
        Commands.runOnce(intake::incrementRollerVelocityByOne)
    );

    oi.getDecrementRollerVelocityButton().onTrue(Commands.runOnce(intake::decrementRollerVelocityByOne));

    FaultReporter.getInstance()
        .registerSystemCheck(
            IntakeConstants.SUBSYSTEM_NAME,
            intake.getSystemCheckCommand(),
            oi.getIntakeSystemTest());
  }
}
