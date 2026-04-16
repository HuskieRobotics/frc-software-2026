package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.MathUtils;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;

  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private boolean systemTestRunning = false;

  private int fuelCount = 0;
  private boolean previousHasFuel = false;
  private int simulatedFuelCounter = 0;

  // testing mode creation of variables
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Shooter/TestingMode", 0);
  private final LoggedTunableNumber flyWheelLeadVelocity =
      new LoggedTunableNumber("Shooter/FlyWheelLead Velocity RPS", 0);
  private final LoggedTunableNumber flywheelLeadCurrent =
      new LoggedTunableNumber("Shooter/FlywheelLead Current", 0);
  private final LoggedTunableNumber turretPosition =
      new LoggedTunableNumber("Shooter/Turret Position Degrees", 0);
  private final LoggedTunableNumber turretVoltage =
      new LoggedTunableNumber("Shooter/Turret Voltage", 0);
  private final LoggedTunableNumber hoodPosition =
      new LoggedTunableNumber("Shooter/Hood Position Degrees", 0);
  private final LoggedTunableNumber hoodVoltage =
      new LoggedTunableNumber("Shooter/Hood Voltage", 0);

  private final Debouncer hoodAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer turretAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer flywheelAtSetpointDebouncer = new Debouncer(0.1);

  private final Debouncer fuelDetectedDebouncer = new Debouncer(0.75, DebounceType.kFalling);

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
              output -> io.setFlywheelCurrent(output.in(Volts)),
              null,
              this)); // treat volts as amps

  private final SysIdRoutine hoodIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setHoodVoltage(output.in(Volts)), null, this));

  private final SysIdRoutine turretIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setTurretVoltage(output.in(Volts)), null, this));

  public Shooter(ShooterIO io) {
    if (io == null) {
      throw new IllegalArgumentException("ShooterIO cannot be null");
    }
    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Flywheel Lead Current", flywheelIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Hood Voltage", hoodIdRoutine);
    SysIdRoutineChooser.getInstance().addOption("Turret Voltage", turretIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);

    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);

    // the jam detectors must be updated every cycle in order to function properly
    hoodJamDetector.update(shooterInputs.hoodStatorCurrent);
    turretJamDetector.update(shooterInputs.turretStatorCurrent);

    if (Constants.getMode() == Constants.Mode.SIM) {
      simulatedFuelCounter++;
      if (simulatedFuelCounter >= 5) { // every 1/10 second (5 cycles)
        fuelCount++;
        simulatedFuelCounter = 0;
      }
    }

    if (shooterInputs.fuelDetectorHasFuel && !previousHasFuel) {
      fuelCount++;
    }
    previousHasFuel = shooterInputs.fuelDetectorHasFuel;
    fuelDetectedDebouncer.calculate(shooterInputs.fuelDetectorHasFuel);

    if (testingMode.get() == 1) {
      // Flywheel Lead
      if (flyWheelLeadVelocity.get() != 0) {
        io.setFlywheelVelocity(flyWheelLeadVelocity.get());
      } else if (flywheelLeadCurrent.get() != 0) {
        io.setFlywheelCurrent(flywheelLeadCurrent.get());
      }

      // Turret
      if (turretPosition.get() != 0) {
        io.setTurretPosition(Units.degreesToRotations(turretPosition.get()));
      } else if (turretVoltage.get() != 0) {
        io.setTurretVoltage(turretVoltage.get());
      }

      // Hood
      if (hoodPosition.get() != 0) {
        io.setHoodPosition(Units.degreesToRotations(hoodPosition.get()));
      } else if (hoodVoltage.get() != 0) {
        io.setHoodVoltage(hoodVoltage.get());
      }
    }

    Transform3d shooterPose =
        new Transform3d(
            Units.inchesToMeters(-4.5),
            Units.inchesToMeters(2.5),
            Units.inchesToMeters(22.0),
            new Rotation3d(
                0.0,
                Units.rotationsToRadians(-shooterInputs.hoodPositionRot),
                Units.rotationsToRadians(shooterInputs.turretPositionRot)));
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/pose",
        new Pose3d(RobotOdometry.getInstance().getEstimatedPose()).plus(shooterPose));
    Logger.recordOutput(SUBSYSTEM_NAME + "/fuelCount", fuelCount);

    LoggedTracer.record("Shooter");
  }

  public boolean isSystemTestRunning() {
    return this.systemTestRunning;
  }

  public Command getShooterSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> this.systemTestRunning = true),
            getTestVelocityCommand(),
            getTestPositionCommand())
        // .until(
        //     () ->
        //         !FaultReporter.getInstance()
        //             .getFaults(SUBSYSTEM_NAME)
        //             .isEmpty()) // .until() stops the sequence of commands, if it is triggered
        // true,
        // so if isEmpty returns false, that gets negated by the ! and
        // becomes true.
        // Then the .until() would stop the rest of the commands from running, because a fault has
        // been detected. Otherwise, if .until() evaluates to false, then the system check keeps
        // running.
        .andThen(Commands.runOnce(() -> this.systemTestRunning = false));
  }

  public Command getTestVelocityCommand() {
    return Commands.sequence(
        // check if the velocity is at setpoint 1
        Commands.runOnce(() -> io.setFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_1_RPS), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(() -> this.checkFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_1_RPS), this),
        // check if the velocity is at setpoint 2
        Commands.runOnce(() -> io.setFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_2_RPS), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(() -> this.checkFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_2_RPS), this),

        // check if the velocity is at setpoint 3
        Commands.runOnce(() -> io.setFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_3_RPS), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(() -> this.checkFlywheelVelocity(FLYWHEEL_VELOCITY_SETPOINT_3_RPS), this));
  }

  public Command getTestPositionCommand() {
    return Commands.sequence(
        // check if hood and turret are at setpoint 1
        Commands.runOnce(
            () -> io.setHoodPosition(Units.degreesToRotations(HOOD_SETPOINT_1_DEGREES)), this),
        Commands.runOnce(
            () -> io.setTurretPosition(Units.degreesToRotations(TURRET_SETPOINT_1_DEGREES)), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () -> this.checkPosition(HOOD_SETPOINT_1_DEGREES, TURRET_SETPOINT_1_DEGREES), this),
        // check if hood and turret are at setpoint 2
        Commands.runOnce(
            () -> io.setHoodPosition(Units.degreesToRotations(HOOD_SETPOINT_2_DEGREES)), this),
        Commands.runOnce(
            () -> io.setTurretPosition(Units.degreesToRotations(TURRET_SETPOINT_2_DEGREES)), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () -> this.checkPosition(HOOD_SETPOINT_2_DEGREES, TURRET_SETPOINT_2_DEGREES), this),

        // check if hood and turret are at setpoint 3
        Commands.runOnce(
            () -> io.setHoodPosition(Units.degreesToRotations(HOOD_SETPOINT_3_DEGREES)), this),
        Commands.runOnce(
            () -> io.setTurretPosition(Units.degreesToRotations(TURRET_SETPOINT_3_DEGREES)), this),
        Commands.waitSeconds(COMMAND_WAIT_TIME_SECONDS),
        Commands.runOnce(
            () -> this.checkPosition(HOOD_SETPOINT_3_DEGREES, TURRET_SETPOINT_3_DEGREES), this));
  }

  public void checkFlywheelVelocity(double flywheelTargetVelocityRPS) {
    if (!MathUtils.isNear(
        shooterInputs.flywheelLeadVelocityRPS, flywheelTargetVelocityRPS, VELOCITY_TOLERANCE_RPS)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel lead is out of tolerance, should be "
                  + flywheelTargetVelocityRPS
                  + " RPS but is "
                  + shooterInputs.flywheelLeadVelocityRPS
                  + " RPS");
    }

    double followVelocityRPS = shooterInputs.flywheelFollow1VelocityRPS;
    if (followVelocityRPS < 0.0) {
      followVelocityRPS = -followVelocityRPS;
    }

    if (!MathUtils.isNear(followVelocityRPS, flywheelTargetVelocityRPS, VELOCITY_TOLERANCE_RPS)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel follow 1 is out of tolerance, should be "
                  + flywheelTargetVelocityRPS
                  + " RPS but is "
                  + followVelocityRPS
                  + " RPS");
    }

    followVelocityRPS = shooterInputs.flywheelFollow2VelocityRPS;
    if (followVelocityRPS < 0.0) {
      followVelocityRPS = -followVelocityRPS;
    }

    if (!MathUtils.isNear(followVelocityRPS, flywheelTargetVelocityRPS, VELOCITY_TOLERANCE_RPS)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "flywheel follow 2 is out of tolerance, should be "
                  + flywheelTargetVelocityRPS
                  + " RPS but is "
                  + followVelocityRPS
                  + " RPS");
    }
  }

  public void checkPosition(
      double hoodIntendedPositionDegrees, double turretIntendedPositionDegrees) {
    // Check if hood position is where it should be
    if (!MathUtils.isNear(
        shooterInputs.hoodPositionRot,
        Units.degreesToRotations(hoodIntendedPositionDegrees),
        HOOD_TOLERANCE_ANGLE_ROT)) {

      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Hood position is out of tolerance, should be "
                  + hoodIntendedPositionDegrees
                  + " but is "
                  + Units.rotationsToDegrees(shooterInputs.hoodPositionRot));
    }

    // Check if turret position is where it should be
    if (!MathUtils.isNear(
        shooterInputs.turretPositionRot,
        Units.degreesToRotations(turretIntendedPositionDegrees),
        TURRET_TOLERANCE_ANGLE_ROT)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Turret position is out of tolerance, should be "
                  + turretIntendedPositionDegrees
                  + " but is "
                  + Units.rotationsToDegrees(shooterInputs.turretPositionRot));
    }
  }

  public boolean isHoodAtSetPoint() {
    return hoodAtSetpointDebouncer.calculate(
        MathUtils.isNear(
            shooterInputs.hoodPositionRot,
            shooterInputs.hoodReferencePositionRot,
            HOOD_TOLERANCE_ANGLE_ROT));
  }

  public boolean isTurretAtSetPoint() {

    return turretAtSetpointDebouncer.calculate(
        MathUtils.isNear(
            shooterInputs.turretPositionRot,
            shooterInputs.turretReferencePositionRot,
            TURRET_TOLERANCE_ANGLE_ROT));
  }

  public boolean isFlywheelAtSetPoint() {
    return flywheelAtSetpointDebouncer.calculate(
        MathUtils.isNear(
            shooterInputs.flywheelLeadVelocityRPS,
            shooterInputs.flywheelLeadReferenceVelocityRPS,
            VELOCITY_TOLERANCE_RPS));
  }

  public void setFlywheelVelocity(double velocityRPS) {
    io.setFlywheelVelocity(velocityRPS);
  }

  public void setTurretPosition(double positionRot) {

    double degrees = Units.rotationsToDegrees(positionRot);

    degrees = ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

    if (degrees < Units.rotationsToDegrees(TURRET_LOWER_ANGLE_LIMIT_ROT)
        || degrees > Units.rotationsToDegrees(TURRET_UPPER_ANGLE_LIMIT_ROT)) {
      return;
    } else {
      io.setTurretPosition(Units.degreesToRotations(degrees));
    }
  }

  public void setTurretLockPositionForBank() {
    io.setTurretPosition(TURRET_LOCK_POSITION_ROT);
  }

  public void setHoodPosition(double positionRot) {
    io.setHoodPosition(positionRot);
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
    io.setHoodVoltage(0.0);
  }

  public void zeroHood() {
    io.zeroHoodPosition();
  }

  public void zeroTurret() {
    io.zeroTurretPosition();
  }

  public double getTurretAngularVelocityRPS() {
    return shooterInputs.turretVelocityRPS;
  }

  public double getTurretPositionRot() {
    return shooterInputs.turretPositionRot;
  }

  public double getTurretReferencePositionRot() {
    return shooterInputs.turretReferencePositionRot;
  }

  public double getFlywheelLeadVelocityRPS() {
    return shooterInputs.flywheelLeadVelocityRPS;
  }

  public boolean getFuelDetected() {
    return fuelDetectedDebouncer.calculate(shooterInputs.fuelDetectorHasFuel);
  }

  public boolean hasFuel() {
    return shooterInputs.fuelDetectorHasFuel;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public void resetFuelCount() {
    fuelCount = 0;
    previousHasFuel = false;
  }
}
