package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.MathUtils;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

  private double deployerLinearPositionMeters = 0.0;
  private boolean inDeployedState = false;
  private boolean areRollersActiveState = false;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final CurrentSpikeDetector rollerJamDetector =
      new CurrentSpikeDetector(
          IntakeConstants.ROLLER_JAMMED_CURRENT_AMPS,
          IntakeConstants.ROLLER_JAMMED_TIME_THRESHOLD_SECONDS);

  private final Alert rollerJamAlert = new Alert("Intake roller jammed", Alert.AlertType.kError);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Intake/TestingMode", 0);
  private final LoggedTunableNumber rollerVelocityRPS =
      new LoggedTunableNumber("Intake/RollerVelocityRPS", 0.0);
  private final LoggedTunableNumber rollerCurrent =
      new LoggedTunableNumber("Intake/RollerCurrent", 0.0);

  private final LoggedTunableNumber deployerPositionRotations =
      new LoggedTunableNumber("Intake/DeployerPositionRotations", 0.0);

  private final LoggedTunableNumber deployerPositionLinearInches =
      new LoggedTunableNumber("Intake/DeployerPositionLinearInches", 0.0);

  private final LoggedTunableNumber deployerVoltage =
      new LoggedTunableNumber("Intake/DeployerVoltage", 0.0);
  private final LoggedTunableNumber deployerJostleFuelCurrent =
      new LoggedTunableNumber("Intake/DeployerCurrent", DEPLOYER_JOSTLE_FUEL_CURRENT);

  private final Debouncer deployerDeployedDebouncer = new Debouncer(0.2);
  private final Debouncer deployerRetractedDebouncer = new Debouncer(0.2);
  private final Debouncer rollerAtSetPointDebouncer = new Debouncer(0.1);

  private final SysIdRoutine rollerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> intakeIO.setRollerCurrent(output.in(Volts)),
              null,
              this)); // treat volts as amps

  private final SysIdRoutine deployerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> intakeIO.setDeployerVoltage(output.in(Volts)), null, this));

  public Intake(IntakeIO io) {
    this.intakeIO = io;

    SysIdRoutineChooser.getInstance().addOption("Roller Current", rollerSysIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Deployer Voltage", deployerSysIdRoutine);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {

      if (deployerVoltage.get() != 0) {
        intakeIO.setDeployerVoltage(deployerVoltage.get());
      } else if (deployerPositionRotations.get() != 0) {
        intakeIO.setDeployerPosition(deployerPositionRotations.get());
      } else if (deployerPositionLinearInches.get() != 0) {
        setLinearPosition(Units.inchesToMeters(deployerPositionLinearInches.get()));
      }

      if (rollerVelocityRPS.get() != 0) {
        intakeIO.setRollerVelocity(rollerVelocityRPS.get());
      } else if (rollerCurrent.get() != 0) {
        intakeIO.setRollerCurrent(rollerCurrent.get());
      }
    }

    // checkRollerJam();

    // update debouncer objects; this must be done every cycle
    rollerAtSetPointDebouncer.calculate(
        MathUtils.isNear(
            inputs.rollerVelocityRPS, ROLLER_TARGET_VELOCITY_RPS, ROLLER_VELOCITY_TOLERANCE_RPS));
    deployerDeployedDebouncer.calculate(
        MathUtils.isNear(
            this.deployerLinearPositionMeters,
            DEPLOYED_LINEAR_POSITION_METERS,
            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS));
    deployerRetractedDebouncer.calculate(
        MathUtils.isNear(
            this.deployerLinearPositionMeters,
            RETRACTED_LINEAR_POSITION_METERS,
            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS));

    this.deployerLinearPositionMeters =
        DEPLOYER_CIRCUMFERENCE_METERS * inputs.deployerAngularPositionRot;

    Logger.recordOutput(SUBSYSTEM_NAME + "/DeployerLinearPosition", deployerLinearPositionMeters);
    LoggedTracer.record("Intake");
  }

  public void setLinearPosition(double linearPositionMeters) {
    double angularPositionRot = linearPositionMeters / DEPLOYER_CIRCUMFERENCE_METERS;
    intakeIO.setDeployerPosition(angularPositionRot);
  }

  private void checkRollerJam() {
    if (rollerJamDetector.update(Math.abs(inputs.rollerStatorCurrent))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                      Commands.runOnce(this::outTakeRoller, this),
                      Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.INTAKE_JAMMED))
                          .withTimeout(ROLLER_UNJAM_DURATION_SECONDS),
                      Commands.runOnce(this::startRoller, this))
                  .withName("Stop Intake Jammed"));
      rollerJamAlert.set(true);
    } else {
      rollerJamAlert.set(false);
    }
  }

  public void startRoller() {
    areRollersActiveState = true;
    intakeIO.setRollerVelocity(IntakeConstants.ROLLER_TARGET_VELOCITY_RPS);
  }

  public void reverseRoller() {
    intakeIO.setRollerVelocity(ROLLER_TARGET_VELOCITY.unaryMinus().div(2));
  }

  public void startRollerInAuto() {
    areRollersActiveState = true;
    intakeIO.setRollerVelocity(IntakeConstants.ROLLER_AUTO_TARGET_VELOCITY_RPS);
  }

  public void stopRoller() {
    areRollersActiveState = false;
    intakeIO.setRollerVelocity(0.0);
  }

  public void outTakeRoller() {
    areRollersActiveState = true;
    intakeIO.setRollerVelocity(IntakeConstants.ROLLER_EJECT_VELOCITY_RPS);
  }

  public void deployIntake() {
    inDeployedState = true;
    setLinearPosition(DEPLOYED_LINEAR_POSITION_METERS);
  }

  public void retractIntake() {
    inDeployedState = false;
    setLinearPosition(RETRACTED_LINEAR_POSITION_METERS);
  }

  public void jostleFuelIn() {
    inDeployedState = false;
    if (this.deployerLinearPositionMeters > DEPLOYER_HOPPER_INTERFERENCE_LIMIT_METERS) {
      // only jostle if we're far enough away from the hopper to not cause interference
      intakeIO.setDeployerCurrent(deployerJostleFuelCurrent.get());
    } else {
      intakeIO.setDeployerCurrent(0.0);
    }
  }

  public void jostleFuelOut() {
    inDeployedState = false;
    if (this.deployerLinearPositionMeters < DEPLOYED_LINEAR_POSITION_METERS) {
      // only jostle if we're far enough away from the hopper to not cause interference
      intakeIO.setDeployerCurrent(-deployerJostleFuelCurrent.get());
    } else {
      intakeIO.setDeployerCurrent(0.0);
    }
  }

  public Command getDeployAndStartCommand() {
    return Commands.sequence(
        Commands.runOnce(this::deployIntake, this),
        Commands.waitUntil(this::isDeployed),
        Commands.runOnce(this::startRoller, this));
  }

  public Command getDeployAndStartInAutoCommand() {
    return Commands.sequence(
        Commands.runOnce(this::deployIntake, this),
        Commands.waitUntil(this::isDeployed),
        Commands.runOnce(this::startRollerInAuto, this));
  }

  public Command getRetractAndStopCommand() {
    return Commands.sequence(
        Commands.runOnce(this::stopRoller, this),
        Commands.runOnce(this::retractIntake, this),
        Commands.waitUntil(this::isRetracted));
  }

  public double getPositionMeters() {
    return this.deployerLinearPositionMeters;
  }

  public boolean isDeployed() {
    return deployerDeployedDebouncer.calculate(
        MathUtils.isNear(
            this.deployerLinearPositionMeters,
            DEPLOYED_LINEAR_POSITION_METERS,
            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS));
  }

  public boolean isRetracted() {
    return deployerRetractedDebouncer.calculate(
        MathUtils.isNear(
            this.deployerLinearPositionMeters,
            RETRACTED_LINEAR_POSITION_METERS,
            DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS));
  }

  public boolean inDeployedState() {
    return inDeployedState;
  }

  public boolean areRollersActive() {
    return areRollersActiveState;
  }

  public Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(this::deployIntake, this),
            Commands.waitSeconds(2.0)
                .andThen(
                    () -> {
                      if (!MathUtils.isNear(
                          inputs.deployerAngularPositionRot,
                          DEPLOYED_ANGULAR_POSITION_ROT,
                          DEPLOYER_ANGULAR_POSITION_TOLERANCE_ROT)) {
                        FaultReporter.getInstance()
                            .addFault(
                                SUBSYSTEM_NAME,
                                "Deployer failed to reach deployed position in System Check; was: "
                                    + inputs.deployerAngularPositionRot
                                    + " rotations");
                      }
                    }),
            Commands.runOnce(this::startRoller, this),
            Commands.waitSeconds(0.5)
                .andThen(
                    () -> {
                      if (!MathUtils.isNear(
                          inputs.rollerVelocityRPS,
                          ROLLER_TARGET_VELOCITY_RPS,
                          ROLLER_VELOCITY_TOLERANCE_RPS)) {
                        FaultReporter.getInstance()
                            .addFault(
                                SUBSYSTEM_NAME,
                                "Roller failed to reach target velocity in System Check; was: "
                                    + inputs.rollerVelocityRPS
                                    + " RPS");
                      }
                    }),
            Commands.runOnce(this::outTakeRoller, this),
            Commands.waitSeconds(0.5)
                .andThen(
                    () -> {
                      if (!MathUtils.isNear(
                          inputs.rollerVelocityRPS,
                          ROLLER_EJECT_VELOCITY_RPS,
                          ROLLER_VELOCITY_TOLERANCE_RPS)) {
                        FaultReporter.getInstance()
                            .addFault(
                                SUBSYSTEM_NAME,
                                "Roller failed to reach eject velocity in System Check; was: "
                                    + inputs.rollerVelocityRPS
                                    + " RPS");
                      }
                    }),
            Commands.runOnce(this::stopRoller, this),
            Commands.runOnce(this::retractIntake, this),
            Commands.waitSeconds(2.0)
                .andThen(
                    () -> {
                      if (!MathUtils.isNear(
                          inputs.deployerAngularPositionRot,
                          RETRACTED_ANGULAR_POSITION_ROT,
                          DEPLOYER_ANGULAR_POSITION_TOLERANCE_ROT)) {
                        FaultReporter.getInstance()
                            .addFault(
                                SUBSYSTEM_NAME,
                                "Deployer failed to reach retracted position in System Check; was: "
                                    + inputs.deployerAngularPositionRot
                                    + " rotations");
                      }
                    }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(
            Commands.sequence(
                Commands.runOnce(this::stopRoller, this),
                Commands.runOnce(this::retractIntake, this)));
  }
}
