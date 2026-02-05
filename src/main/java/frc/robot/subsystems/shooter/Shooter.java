package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  // all subsystems receive a reference to their IO implementation when constructed
  private ShooterIO io;

  // all subsystems create the IO inputs instance for this subsystem
  private final ShooterIOInputsAutoLogged shooterInputs =
      new ShooterIOInputsAutoLogged(); // error will be removed once we build the code

  // When initially testing a mechanism, it is best to manually provide a voltage or current to
  // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
  // when multiple motors are involved, that is not possible. Using a tunables to enable testing
  // mode and, for the shooter, specifying current or velocity is convenient. This feature is also
  // an efficient approach when, for example, empirically tuning the velocity for different
  // distances when shooting a game piece.

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

  // The SysId routine is used to characterize the mechanism. While the SysId routine is intended to
  // be used for voltage control, we can apply a current instead and reinterpret the units when
  // performing the analysis in SysId.

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
    this.io = io;

    // Register this subsystem's SysId routine with the SysIdRoutineChooser. This allows
    // the routine to be selected and executed from the dashboard.
    SysIdRoutineChooser.getInstance().addOption("Flywheel Lead Current", flywheelIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Hood Voltage", hoodIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Turret Voltage", turretIdRoutine);

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getShooterSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);

    if (testingMode.get() == 1) { // If we are te
      // Flywheel Lead
      if (flyWheelLeadVelocity.get() != 0) {
        io.setFlywheelVelocity(RotationsPerSecond.of(flyWheelLeadVelocity.get()));
      } else if (flywheelLeadCurrent.get() != 0) {
        io.setFlywheelCurrent(Amps.of(flywheelLeadCurrent.get()));
      } else if (turretPosition.get() != 0) {
        io.setTurretPosition(Degrees.of(turretPosition.get()));
      } else if (turretVoltage.get() != 0) {
        io.setTurretVoltage(Volts.of(turretVoltage.get()));
      } else if (hoodPosition.get() != 0) {
        io.setHoodPosition(Degrees.of(hoodPosition.get()));
      } else if (hoodVoltage.get() != 0) {
        io.setHoodVoltage(Volts.of(hoodVoltage.get()));
      }
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Shooter");
  }

  public Command getShooterSystemCheckCommand() {
    return Commands.sequence(getTestVelocityCommand(), getTestPositionCommand())
        .until(() -> (!FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty()))
        .andThen(
            Commands.runOnce(
                () -> { // FIXME: set to safe positions/velocities
                  io.setFlywheelVelocity(RotationsPerSecond.of(5));
                  io.setHoodPosition(Degrees.of(30));
                  io.setTurretPosition(Degrees.of(30));
                }));
  }

  public Command getTestVelocityCommand() {
    return Commands.sequence(
        // check if the velocity is at setpoint 1
        Commands.runOnce(
            () ->
                io.setFlywheelVelocity(
                    RotationsPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS))),
        Commands.waitSeconds(3),
        Commands.runOnce(
            () ->
                this.checkFlywheelVelocity(
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_1_RPS)),

        // check if the velocity is at setpoint 2
        Commands.runOnce(
            () ->
                io.setFlywheelVelocity(
                    RotationsPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () ->
                this.checkFlywheelVelocity(
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_2_RPS)),

        // check if the velocity is at setpoint 3
        Commands.runOnce(
            () ->
                io.setFlywheelVelocity(
                    RotationsPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS))),
        Commands.waitSeconds(ShooterConstants.COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () ->
                this.checkFlywheelVelocity(
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS,
                    ShooterConstants.FLYWHEEL_VELOCITY_SETPOINT_3_RPS)));
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

  public void checkFlywheelVelocity(
      double flywheelLeadIntendedVelocityRPS,
      double flywheelFollow1IntendedVelocityRPS,
      double flywheelFollow2IntendedVelocityRPS) {
    // flywheel lead
    if (Math.abs(
            shooterInputs.flywheelLeadVelocity.in(RotationsPerSecond)
                - flywheelLeadIntendedVelocityRPS)
        > VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
      if (Math.abs(shooterInputs.flywheelLeadVelocity.in(RotationsPerSecond))
              - Math.abs(flywheelLeadIntendedVelocityRPS)
          < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel lead velocity is too low, should be "
                    + flywheelLeadIntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelLeadVelocity);
      } else if (Math.abs(shooterInputs.flywheelLeadVelocity.in(RotationsPerSecond))
              - Math.abs(flywheelLeadIntendedVelocityRPS)
          > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel lead velocity is too high, should be "
                    + flywheelLeadIntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelLeadVelocity);
      }
    }

    // flywheel follow 1
    if (Math.abs(
            shooterInputs.flywheelFollow1Velocity.in(RotationsPerSecond)
                - flywheelFollow1IntendedVelocityRPS)
        > VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
      if (Math.abs(shooterInputs.flywheelFollow1Velocity.in(RotationsPerSecond))
              - Math.abs(flywheelFollow1IntendedVelocityRPS)
          < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel follow 1 velocity is too low, should be "
                    + flywheelFollow1IntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelFollow1Velocity);
      } else if (Math.abs(shooterInputs.flywheelFollow1Velocity.in(RotationsPerSecond))
              - Math.abs(flywheelFollow1IntendedVelocityRPS)
          > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel follow 1 velocity is too high, should be "
                    + flywheelFollow1IntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelFollow1Velocity);
      }
    }
    // flywheel follow 2
    if (Math.abs(
            shooterInputs.flywheelFollow2Velocity.in(RotationsPerSecond)
                - flywheelFollow2IntendedVelocityRPS)
        > VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
      if (Math.abs(shooterInputs.flywheelFollow2Velocity.in(RotationsPerSecond))
              - Math.abs(flywheelFollow2IntendedVelocityRPS)
          < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel follow 2 velocity is too low, should be "
                    + flywheelFollow2IntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelFollow2Velocity);
      } else if (Math.abs(shooterInputs.flywheelFollow2Velocity.in(RotationsPerSecond))
              - Math.abs(flywheelFollow2IntendedVelocityRPS)
          > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Flywheel follow 2 velocity is too high, should be "
                    + flywheelFollow2IntendedVelocityRPS
                    + " but is "
                    + shooterInputs.flywheelFollow2Velocity);
      }
    }
  }

  public void checkPosition(
      double hoodIntendedPositionDegrees, double turretIntendedPositionDegrees) {
    // Check if hood position is where it should be
    if (Math.abs(shooterInputs.hoodPosition.in(Degrees) - hoodIntendedPositionDegrees)
        > POSITION_TOLERANCE) {
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
        > POSITION_TOLERANCE) {
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

  // public void reverseShooter(double reverseAmps) { //FIXME: might not be needed
  //  io.setFlywheelTorqueCurrent(-ShooterConstants.SHOOTER_CURRENT);
  // }

  public void setFlywheelLeadVelocity(AngularVelocity velocity) {
    io.setFlywheelVelocity(velocity);
  }

  public void
      setIdleVelocity() { // when shooting modes are NOT determining the necessary velocity for the
    // flywheels
    io.setFlywheelVelocity(RotationsPerSecond.of(ShooterConstants.SHOOTER_IDLE_VELOCITY_RPS));
  }

  public void setTurretPosition(Angle position) {
    io.setTurretPosition(position);
  }

  public void setHoodPosition(Angle position) {
    io.setHoodPosition(position);
  }

  public void setHoodVoltage(Voltage voltage) {
    io.setHoodVoltage(voltage);
  }
}
