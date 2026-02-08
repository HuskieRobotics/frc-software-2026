package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.hopper.HopperConstants.*;
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

  private Alert spindexerJammedAlert = new Alert("Spindexer jam detected.", AlertType.kError);
  private Alert kickerJammedAlert =
      new Alert(
          "Kicker jam detected. This is the motor that kicks fuel from hopper into shooter.",
          AlertType.kError);

  public Hopper(HopperIO io) {
    this.io = io;

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME,
    // getHopperSystemCheckCommand());
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
      } else if (kickerVelocityRPS.get() != 0) {
        io.setKickerVelocity(RotationsPerSecond.of(kickerVelocityRPS.get()));
      } else if (kickerCurrent.get() != 0) {
        io.setKickerCurrent(Amps.of(kickerCurrent.get()));
      }
    }

    if (spindexerSpikeDetector.update(Math.abs(inputs.spindexerStatorCurrent.in(Amps)))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                  Commands.runOnce(() -> io.setSpindexerVelocity(SPINDEXER_UNJAM_VELOCITY))
                      .withName("spindexer jammed")));
      spindexerJammedAlert.set(true);
    } else {
      spindexerJammedAlert.set(false);
    }

    if (kickerSpikeDetector.update(Math.abs(inputs.kickerStatorCurrent.in(Amps)))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                  Commands.runOnce(() -> io.setKickerVelocity(KICKER_UNJAM_VELOCITY))
                      .withName("kicker jammed")));
      kickerJammedAlert.set(true);
    } else {
      kickerJammedAlert.set(false);
    }

    LoggedTracer.record(SUBSYSTEM_NAME);
  }

  /*  private Command
      getHopperSystemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                io.setKickerVelocity(
                    RotationsPerSecond.of(KICKER_MOTOR_MANUAL_CONTROL_VELOCITY))), // FIXME
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if (inputs.kickerVelocity.lt(kickerVelocity)) {
                FaultReporter.getInstance()
                    .addFault(SUBSYSTEM_NAME, "kicker not moving as fast", false);
              }
            }),
        Commands.runOnce(
            () ->
                io.setSpindexerVelocity(
                    RotationsPerSecond.of(SPINDEXER_MOTOR_MANUAL_CONTROL_VELOCITY))),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
                () -> {
                  if (inputs.spindexerVelocity.lt(spindexerVelocity)) {
                    FaultReporter.getInstance()
                        .addFault(SUBSYSTEM_NAME, "spindexer not moving as fast", false);
                  }
                })
            .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
            .andThen(Commands.runOnce(() -> io.setKickerVelocity(RotationsPerSecond.of(0.0)))));
  } */

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

  public double getKickerVelocityRPS() {
    return kickerVelocityRPS.get();
  }

  public double getSpindexerVelocityRPS() {
    return spindexerVelocityRPS.get();
  }
}
