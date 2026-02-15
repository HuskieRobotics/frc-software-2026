package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;

  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  // testing mode creation of variables
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Shooter/TestingMode", 0);
  private final LoggedTunableNumber flyWheelLeadVelocity =
      new LoggedTunableNumber("Shooter/FlyWheelLead Velocity", 0);
  private final LoggedTunableNumber flywheelLeadCurrent =
      new LoggedTunableNumber("Shooter/FlywheelLead Current", 0);
  private final LoggedTunableNumber turretPosition =
      new LoggedTunableNumber("Shooter/Turret Position", 0);
  private final LoggedTunableNumber turretVoltage =
      new LoggedTunableNumber("Shooter/Turret Voltage", 0);
  private final LoggedTunableNumber hoodPosition =
      new LoggedTunableNumber("Shooter/Hood Position", 0);
  private final LoggedTunableNumber hoodVoltage =
      new LoggedTunableNumber("Shooter/Hood Voltage", 0);

  private final Debouncer hoodAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer turretAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer flywheelAtSetpointDebouncer = new Debouncer(0.1);

  private CurrentSpikeDetector hoodJamDetector =
      new CurrentSpikeDetector(HOOD_CURRENT_THRESHOLD_AMPS, HOOD_CURRENT_TIME_THRESHOLD_SECONDS);

  private CurrentSpikeDetector turretJamDetector =
      new CurrentSpikeDetector(
          TURRET_CURRENT_THRESHOLD_AMPS, TURRET_CURRENT_TIME_THRESHOLD_SECONDS);

  private final SysIdRoutine flywheelIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(10), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setFlywheelCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps

  private final SysIdRoutine hoodIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setHoodVoltage(output), null, this));

  private final SysIdRoutine turretIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setTurretVoltage(output), null, this));

  public Shooter(ShooterIO io) {
    if (io == null) {
      throw new IllegalArgumentException("ShooterIO cannot be null");
    }
    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Flywheel Lead Current", flywheelIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Hood Voltage", hoodIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Turret Voltage", turretIdRoutine);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getShooterSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);

    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);

    // the jam detectors must be updated every cycle in order to function properly
    hoodJamDetector.update(shooterInputs.hoodStatorCurrent.in(Amps));
    turretJamDetector.update(shooterInputs.turretStatorCurrent.in(Amps));

    if (testingMode.get() == 1) {
      // Flywheel Lead
      if (flyWheelLeadVelocity.get() != 0) {
        io.setFlywheelVelocity(RotationsPerSecond.of(flyWheelLeadVelocity.get()));
      } else if (flywheelLeadCurrent.get() != 0) {
        io.setFlywheelCurrent(Amps.of(flywheelLeadCurrent.get()));
      }

      // Turret
      if (turretPosition.get() != 0) {
        io.setTurretPosition(Degrees.of(turretPosition.get()));
      } else if (turretVoltage.get() != 0) {
        io.setTurretVoltage(Volts.of(turretVoltage.get()));
      }

      // Hood
      // Hood
      if (hoodPosition.get() != 0) {
        io.setHoodPosition(Degrees.of(hoodPosition.get()));
      } else if (hoodVoltage.get() != 0) {
        io.setHoodVoltage(Volts.of(hoodVoltage.get()));
      }
    }

    Transform3d shooterPose =
        new Transform3d(
            Units.inchesToMeters(-4.5),
            Units.inchesToMeters(2.5),
            Units.inchesToMeters(22.0),
            new Rotation3d(
                0.0,
                -shooterInputs.hoodPosition.in(Radians),
                shooterInputs.turretPosition.in(Radians)));
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/pose",
        new Pose3d(RobotOdometry.getInstance().getEstimatedPose()).plus(shooterPose));

    LoggedTracer.record("Shooter");
  }

  public Command getShooterSystemCheckCommand() {
    return Commands.sequence(getTestVelocityCommand(), getTestPositionCommand())
        .until(
            () ->
                (FaultReporter.getInstance()
                    .getFaults(SUBSYSTEM_NAME)
                    .isEmpty())) // FIXME: might need to add a ! before the statement
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setFlywheelVelocity(
                      RotationsPerSecond.of(
                          20)); // FIXME: determine necessary velocity for systems check based on
                  // the min and max flywheel velocity
                }));
  }

  public Command getTestVelocityCommand() {
    return Commands.sequence(
        // check if the velocity is at setpoint 1
        Commands.runOnce(
            () -> io.setFlywheelVelocity(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS)),
        Commands.waitSeconds(3),
        Commands.runOnce(
            () -> this.checkFlywheelVelocity(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS)),
        // check if the velocity is at setpoint 2
        Commands.runOnce(
            () -> io.setFlywheelVelocity(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS)),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () -> this.checkFlywheelVelocity(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS)),

        // check if the velocity is at setpoint 3
        Commands.runOnce(
            () -> io.setFlywheelVelocity((ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () -> this.checkFlywheelVelocity(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS)));
  }

  public Command getTestPositionCommand() {
    return Commands.sequence(
        // check if hood and turret are at setpoint 1
        Commands.runOnce(
            () -> io.setHoodPosition(Degrees.of(ShooterConstants.HOOD_SETPOINT_1_DEGREES))),
        Commands.runOnce(
            () -> io.setTurretPosition(Degrees.of(ShooterConstants.TURRET_SETPOINT_1_DEGREES))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () ->
                this.checkPosition(
                    ShooterConstants.HOOD_SETPOINT_1_DEGREES,
                    ShooterConstants.TURRET_SETPOINT_1_DEGREES)),

        // check if hood and turret are at setpoint 2
        Commands.runOnce(
            () -> io.setHoodPosition(Degrees.of(ShooterConstants.HOOD_SETPOINT_2_DEGREES))),
        Commands.runOnce(
            () -> io.setTurretPosition(Degrees.of(ShooterConstants.TURRET_SETPOINT_2_DEGREES))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () ->
                this.checkPosition(
                    ShooterConstants.HOOD_SETPOINT_2_DEGREES,
                    ShooterConstants.TURRET_SETPOINT_2_DEGREES)),

        // check if hood and turret are at setpoint 3
        Commands.runOnce(
            () -> io.setHoodPosition(Degrees.of(ShooterConstants.HOOD_SETPOINT_3_DEGREES))),
        Commands.runOnce(
            () -> io.setTurretPosition(Degrees.of(ShooterConstants.TURRET_SETPOINT_3_DEGREES))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () ->
                this.checkPosition(
                    ShooterConstants.HOOD_SETPOINT_3_DEGREES,
                    ShooterConstants.TURRET_SETPOINT_3_DEGREES)));
  }

  public void checkFlywheelVelocity(AngularVelocity flywheelTargetVelocity) {
    if (shooterInputs.flywheelLeadVelocity.isNear(flywheelTargetVelocity, VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel lead is out of tolerance, should be "
                  + flywheelTargetVelocity.in(RotationsPerSecond)
                  + " RPS but is "
                  + shooterInputs.flywheelLeadVelocity.in(RotationsPerSecond)
                  + " RPS");
    }

    if (shooterInputs.flywheelFollow1Velocity.isNear(flywheelTargetVelocity, VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel follow 1 is out of tolerance, should be "
                  + flywheelTargetVelocity.in(RotationsPerSecond)
                  + " RPS but is "
                  + shooterInputs.flywheelFollow1Velocity.in(RotationsPerSecond)
                  + " RPS");
    }

    if (shooterInputs.flywheelFollow2Velocity.isNear(flywheelTargetVelocity, VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel follow 2 is out of tolerance, should be "
                  + flywheelTargetVelocity.in(RotationsPerSecond)
                  + " RPS but is "
                  + shooterInputs.flywheelFollow2Velocity.in(RotationsPerSecond)
                  + " RPS");
    }
  }

  public void checkPosition(
      double hoodIntendedPositionDegrees, double turretIntendedPositionDegrees) {
    // Check if hood position is where it should be
    if (Math.abs(shooterInputs.hoodPosition.in(Degrees) - hoodIntendedPositionDegrees)
        > ShooterConstants.HOOD_TOLERANCE_ANGLE.in(Degrees)) {
      if (shooterInputs.hoodPosition.in(Degrees) - hoodIntendedPositionDegrees < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Hood position is too low, should be "
                    + hoodIntendedPositionDegrees
                    + " but is "
                    + shooterInputs.hoodPosition);
      } else if (shooterInputs.hoodPosition.in(Degrees) - hoodIntendedPositionDegrees > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Hood position is too high, should be "
                    + hoodIntendedPositionDegrees
                    + " but is "
                    + shooterInputs.hoodPosition);
      }
    }

    // Check if turret position is where it should be
    if (Math.abs(shooterInputs.turretPosition.in(Degrees) - turretIntendedPositionDegrees)
        > ShooterConstants.TURRET_TOLERANCE_ANGLE.in(Degrees)) {
      if (shooterInputs.turretPosition.in(Degrees) - turretIntendedPositionDegrees < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Turret position is too low, should be "
                    + turretIntendedPositionDegrees
                    + " but is "
                    + shooterInputs.turretPosition);
      } else if (shooterInputs.turretPosition.in(Degrees) - turretIntendedPositionDegrees > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Turret position is too high, should be "
                    + turretIntendedPositionDegrees
                    + " but is "
                    + shooterInputs.turretPosition);
      }
    }
  }

  public boolean isHoodAtSetPoint() {
    return hoodAtSetpointDebouncer.calculate(
        shooterInputs.hoodPosition.isNear(
            shooterInputs.hoodReferencePosition, HOOD_TOLERANCE_ANGLE));
  }

  public boolean isTurretAtSetPoint() {

    return turretAtSetpointDebouncer.calculate(
        shooterInputs.turretPosition.isNear(
            shooterInputs.turretReferencePosition, TURRET_TOLERANCE_ANGLE));
  }

  public boolean isFlywheelAtSetPoint() {
    return flywheelAtSetpointDebouncer.calculate(
        shooterInputs.flywheelLeadVelocity.isNear(
            shooterInputs.flywheelLeadReferenceVelocity, VELOCITY_TOLERANCE));
  }

  public void setFlywheelVelocity(AngularVelocity velocity) {
    io.setFlywheelVelocity(velocity);
  }

  public void setTurretPosition(Angle position) {
    io.setTurretPosition(position);
  }

  public void setHoodPosition(Angle position) {
    io.setHoodPosition(position);
  }

  public boolean isHoodJammed() {
    return hoodJamDetector.getAsBoolean();
  }

  public boolean isTurretJammed() {
    return turretJamDetector.getAsBoolean();
  }

  public void lowerHoodSlow() {
    io.lowerHoodSlow(HOOD_SLOW_LOWER_VOLTAGE);
  }

  public void stopHood() {
    io.setHoodVoltage(Volts.of(0.0));
  }

  public void zeroHood() {
    io.zeroHoodPosition();
  }

  public void zeroTurret() {
    io.zeroTurretPosition();
  }
}
