package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants.*;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private HopperIO io;

  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final LoggedTunableNumber spindexerVelocityRPS =
      new LoggedTunableNumber("Hopper/SpindexerVelocity", 0);
  private final LoggedTunableNumber spindexerCurrent =
      new LoggedTunableNumber("Hopper/Spindexer Current", 0);
  private final LoggedTunableNumber kickerVelocityRPS =
      new LoggedTunableNumber("Hopper/KickerVelocity", 0);
  private final LoggedTunableNumber kickerCurrent =
      new LoggedTunableNumber("Hopper/Kicker Current", 0);

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Hopper/TestingMode", 0);

  private CurrentSpikeDetector spindexerSpikeDetector =
      new CurrentSpikeDetector(
          SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS, SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS);

  private CurrentSpikeDetector kickerSpikeDetector =
      new CurrentSpikeDetector(
          KICKER_CURRENT_SPIKE_THRESHOLD_AMPS, KICKER_CURRENT_SPIKE_THRESHOLD_SECONDS);

  private final Debouncer spindexerAtSetpointDebouncer = new Debouncer(0.2);
  private final Debouncer kickerAtSetpointDebouncer = new Debouncer(0.2);

  private Alert spindexerJammedAlert = new Alert("Spindexer jam detected.", AlertType.kError);

  private Alert kickerJammedAlert = new Alert("Kicker jam detected.", AlertType.kError);

  private final SysIdRoutine kickerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setKickerCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps

  private final SysIdRoutine spindexerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              state -> SignalLogger.writeString("SysIdRotationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setSpindexerCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps

  public Hopper(HopperIO io) {
    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Kicker Current", kickerSysIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Spindexer Current", spindexerSysIdRoutine);

    getHopperSystemCheckCommand();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      if (spindexerVelocityRPS.get() != 0) {
        io.setSpindexerVelocity(RotationsPerSecond.of(spindexerVelocityRPS.get()));
      } else if (spindexerCurrent.get() != 0) {
        io.setSpindexerCurrent(Amps.of(spindexerCurrent.get()));
      }

      if (kickerVelocityRPS.get() != 0) {
        io.setKickerVelocity(RotationsPerSecond.of(kickerVelocityRPS.get()));
      } else if (kickerCurrent.get() != 0) {
        io.setKickerCurrent(Amps.of(kickerCurrent.get()));
      }
    }

    if (frc.robot.Constants.getMode() != Mode.SIM) {
      if (spindexerSpikeDetector.update(Math.abs(inputs.spindexerStatorCurrent.in(Amps)))) {
        CommandScheduler.getInstance()
            .schedule(
                Commands.sequence(
                    Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_UNJAM_VELOCITY))
                        .withName("spindexer jammed"),
                    Commands.waitSeconds(SPINDEXER_UNJAM_WAIT_TIME)));
        spindexerJammedAlert.set(true);
      } else {
        spindexerJammedAlert.set(false);
      }

      if (kickerSpikeDetector.update(Math.abs(inputs.kickerStatorCurrent.in(Amps)))) {
        CommandScheduler.getInstance()
            .schedule(
                Commands.sequence(
                    Commands.runOnce(() -> io.setKickerVelocity(KICKER_UNJAM_VELOCITY))
                        .withName("kicker jammed"),
                    Commands.waitSeconds(KICKER_UNJAM_WAIT_TIME)));
        kickerJammedAlert.set(true);
      } else {
        kickerJammedAlert.set(false);
      }
    }

    LoggedTracer.record(SUBSYSTEM_NAME);
  }

  private Command getHopperSystemCheckCommand() {
    return Commands.sequence(getTestVelocityCommand())
        .until(() -> (!FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty()))
        .andThen(
            Commands.runOnce(
                () -> {
                  stopSpindexer();
                  stopKicker();
                }));
  }

  public Command getTestVelocityCommand() {
    return Commands.sequence(
        // check if spindexer velocity is at setpoint 1
        Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_1_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_1_RPS)),

        // check if spindexer velocity is at setpoint 2
        Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_2_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_2_RPS)),

        // check if spindexer velocity is at setpoint 3
        Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_3_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkSpindexerVelocity(SPINDEXER_VELOCITY_SETPOINT_3_RPS)),

        // check if kicker velocity is at setpoint 1
        Commands.runOnce(() -> io.setKickerVelocity(KICKER_VELOCITY_SETPOINT_1_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkKickerVelocity(KICKER_VELOCITY_SETPOINT_1_RPS)),

        // check if kicker velocity is at setpoint 2
        Commands.runOnce(() -> io.setKickerVelocity(KICKER_VELOCITY_SETPOINT_2_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkKickerVelocity(KICKER_VELOCITY_SETPOINT_2_RPS)),

        // check if kicker velocity is at setpoint 3
        Commands.runOnce(() -> io.setKickerVelocity(KICKER_VELOCITY_SETPOINT_3_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(() -> this.checkKickerVelocity(KICKER_VELOCITY_SETPOINT_3_RPS)));
  }

  public void checkSpindexerVelocity(AngularVelocity spindexerTargetVelocity) {
    if (!inputs.spindexerVelocity.isNear(spindexerTargetVelocity, SPINDEXER_VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Spindexer is out of tolerance, should be "
                  + spindexerTargetVelocity.in(RotationsPerSecond)
                  + " RPS but is "
                  + inputs.spindexerVelocity.in(RotationsPerSecond)
                  + " RPS");
    }
  }

  public void checkKickerVelocity(AngularVelocity kickerTargetVelocity) {
    if (!inputs.kickerVelocity.isNear(kickerTargetVelocity, KICKER_VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Kicker is out of tolerance, should be "
                  + kickerTargetVelocity.in(RotationsPerSecond)
                  + " RPS but is "
                  + inputs.kickerVelocity.in(RotationsPerSecond)
                  + " RPS");
    }
  }

  public boolean isSpindexerAtVelocity() {
    return spindexerAtSetpointDebouncer.calculate(
        inputs.spindexerVelocity.isNear(
            inputs.spindexerReferenceVelocity, SPINDEXER_VELOCITY_TOLERANCE));
  }

  public boolean isKickerAtVelocity() {
    return kickerAtSetpointDebouncer.calculate(
        inputs.kickerVelocity.isNear(inputs.kickerReferenceVelocity, KICKER_VELOCITY_TOLERANCE));
  }

  public void setKickerVelocity(AngularVelocity velocity) {
    io.setKickerVelocity(velocity);
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    io.setSpindexerVelocity(velocity);
  }

  public void setKickerCurrent(Current amps) {
    io.setKickerCurrent(amps);
  }

  public void setSpindexerCurrent(Current amps) {
    io.setSpindexerCurrent(amps);
  }

  public AngularVelocity getKickerVelocityRPS() {
    return inputs.kickerVelocity;
  }

  public AngularVelocity getSpindexerVelocityRPS() {
    return inputs.spindexerVelocity;
  }

  public void stop() {
    stopSpindexer();
    stopKicker();
  }

  public void stopSpindexer() {
    io.setSpindexerVelocity(RotationsPerSecond.of(0.0));
  }

  public void stopKicker() {
    io.setKickerVelocity(RotationsPerSecond.of(0.0));
  }
}
