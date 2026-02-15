package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team3061.sim.FlywheelSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  private VelocityTorqueCurrentFOC flywheelLeadVelocityRequest;
  private TorqueCurrentFOC flywheelLeadCurrentRequest;

  private PositionVoltage turretPositionRequest;
  private VoltageOut turretVoltageRequest;

  private PositionVoltage hoodPositionRequest;
  private VoltageOut hoodVoltageRequest;

  // Flywheel Lead Status Signals
  private StatusSignal<Current> flywheelLeadSupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelLeadStatorCurrentStatusSignal;
  private StatusSignal<Current> flywheelLeadTorqueCurrentStatusSignal;

  // Flywheel Follow1 Status Signals
  private StatusSignal<Current> flywheelFollow1SupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow1StatorCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow1TorqueCurrentStatusSignal;

  // Flywheel Follow2 Status Signals
  private StatusSignal<Current> flywheelFollow2SupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow2StatorCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow2TorqueCurrentStatusSignal;
  // Turret, and Hood Status Signals
  private StatusSignal<Current> turretStatorCurrentStatusSignal;
  private StatusSignal<Current> turretSupplyCurrentStatusSignal;
  private StatusSignal<Current> hoodStatorCurrentStatusSignal;
  private StatusSignal<Current> hoodSupplyCurrentStatusSignal;

  // Angular Velocity Status Signals
  // For flywheel lead motor
  private StatusSignal<AngularVelocity> flywheelLeadVelocityStatusSignal;
  private StatusSignal<AngularVelocity> flywheelFollow1VelocityStatusSignal;
  private StatusSignal<AngularVelocity> flywheelFollow2VelocityStatusSignal;

  private StatusSignal<Temperature> flywheelLeadTemperatureStatusSignal;
  private StatusSignal<Temperature> flywheelFollow1TemperatureStatusSignal;
  private StatusSignal<Temperature> flywheelFollow2TemperatureStatusSignal;
  private StatusSignal<Temperature> turretTemperatureStatusSignal;
  private StatusSignal<Temperature> hoodTemperatureStatusSignal;

  private StatusSignal<Voltage> flywheelLeadVoltageStatusSignal;
  private StatusSignal<Voltage> flywheelFollow1VoltageStatusSignal;
  private StatusSignal<Voltage> flywheelFollow2VoltageStatusSignal;
  private StatusSignal<Voltage> turretVoltageStatusSignal;
  private StatusSignal<Voltage> hoodVoltageStatusSignal;

  private StatusSignal<Angle> turretPositionStatusSignal;
  private StatusSignal<Angle> hoodPositionStatusSignal;

  private Angle hoodReferencePosition = Degrees.of(0.0);
  private Angle turretReferencePosition = Degrees.of(0.0);
  private AngularVelocity flywheelLeadReferenceVelocity = RotationsPerSecond.of(0.0);

  private final Debouncer flywheelLeadConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer flywheelFollow1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer flywheelFollow2ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turretConnectedDebouncer = new Debouncer(0.5);

  private TalonFX flywheelLead;
  private TalonFX flywheelFollow1;
  private TalonFX flywheelFollow2;
  private TalonFX turret;
  private TalonFX hood;

  private Alert flywheelLeadConfigAlert =
      new Alert("Failed to apply configuration for fly wheel lead motor.", AlertType.kError);
  private Alert flywheelFollow1ConfigAlert =
      new Alert("Failed to apply configuration for fly wheel follow 1 motor.", AlertType.kError);
  private Alert flywheelFollow2ConfigAlert =
      new Alert("Failed to apply configuration for fly wheel follow 2 motor.", AlertType.kError);
  private Alert hoodConfigAlert =
      new Alert("Failed to apply configuration for hood motor.", AlertType.kError);
  private Alert turretConfigAlert =
      new Alert("Failed to apply configuration for turret motor.", AlertType.kError);

  private final LoggedTunableNumber flywheelLeadKP =
      new LoggedTunableNumber("Shooter/Flywheel kP", ShooterConstants.FLYWHEEL_KP);
  private final LoggedTunableNumber flywheelLeadKI =
      new LoggedTunableNumber("Shooter/Flywheel kI", ShooterConstants.FLYWHEEL_KI);
  private final LoggedTunableNumber flywheelLeadKD =
      new LoggedTunableNumber("Shooter/Flywheel kD", ShooterConstants.FLYWHEEL_KD);
  private final LoggedTunableNumber flywheelLeadKS =
      new LoggedTunableNumber("Shooter/Flywheel kS", ShooterConstants.FLYWHEEL_KS);
  private final LoggedTunableNumber flywheelLeadKV =
      new LoggedTunableNumber("Shooter/Flywheel kV", ShooterConstants.FLYWHEEL_KV);
  private final LoggedTunableNumber flywheelLeadKA =
      new LoggedTunableNumber("Shooter/Flywheel kA", ShooterConstants.FLYWHEEL_KA);
  private final LoggedTunableNumber turretKP =
      new LoggedTunableNumber("Shooter/Turret kP", ShooterConstants.TURRET_ROTATION_KP);
  private final LoggedTunableNumber turretKI =
      new LoggedTunableNumber("Shooter/Turret kI", ShooterConstants.TURRET_ROTATION_KI);
  private final LoggedTunableNumber turretKD =
      new LoggedTunableNumber("Shooter/Turret kD", ShooterConstants.TURRET_ROTATION_KD);
  private final LoggedTunableNumber turretKS =
      new LoggedTunableNumber("Shooter/Turret kS", ShooterConstants.TURRET_ROTATION_KS);
  private final LoggedTunableNumber turretKV =
      new LoggedTunableNumber("Shooter/Turret kV", ShooterConstants.TURRET_ROTATION_KV);
  private final LoggedTunableNumber turretKA =
      new LoggedTunableNumber("Shooter/Turret kA", ShooterConstants.TURRET_ROTATION_KA);
  private final LoggedTunableNumber hoodKP =
      new LoggedTunableNumber("Shooter/Hood kP", ShooterConstants.HOOD_ROTATION_KP);
  private final LoggedTunableNumber hoodKI =
      new LoggedTunableNumber("Shooter/Hood kI", ShooterConstants.HOOD_ROTATION_KI);
  private final LoggedTunableNumber hoodKD =
      new LoggedTunableNumber("Shooter/Hood kD", ShooterConstants.HOOD_ROTATION_KD);
  private final LoggedTunableNumber hoodKS =
      new LoggedTunableNumber("Shooter/Hood kS", ShooterConstants.HOOD_ROTATION_KS);
  private final LoggedTunableNumber hoodKV =
      new LoggedTunableNumber("Shooter/Hood kV", ShooterConstants.HOOD_ROTATION_KV);
  private final LoggedTunableNumber hoodKA =
      new LoggedTunableNumber("Shooter/Hood kA", ShooterConstants.HOOD_ROTATION_KA);

  private FlywheelSystemSim flywheelSim;
  private ArmSystemSim turretLeadSim;
  private ArmSystemSim hoodLeadSim;

  public ShooterIOTalonFX() {

    flywheelLead = new TalonFX(FLYWHEEL_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    flywheelFollow1 =
        new TalonFX(FLYWHEEL_FOLLOW_1_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    flywheelFollow2 =
        new TalonFX(FLYWHEEL_FOLLOW_2_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    turret = new TalonFX(TURRET_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    hood = new TalonFX(HOOD_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    flywheelLeadVelocityRequest = new VelocityTorqueCurrentFOC(0);
    flywheelLeadCurrentRequest = new TorqueCurrentFOC(0.0);

    hoodVoltageRequest = new VoltageOut(0.0);
    hoodPositionRequest = new PositionVoltage(0.0);

    turretVoltageRequest = new VoltageOut(0.0);
    turretPositionRequest = new PositionVoltage(0.0);
    // Set up the flywheels 1 and 2 as followers of the lead flywheel
    flywheelFollow1.setControl(
        new Follower(
            FLYWHEEL_LEAD_MOTOR_ID,
            FLYWHEEL_FOLLOW_1_INVERTED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    // reversed
    flywheelFollow2.setControl(
        new Follower(
            FLYWHEEL_LEAD_MOTOR_ID,
            FLYWHEEL_FOLLOW_2_INVERTED
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    // reversed
    // Assign FLYWHEEL LEAD status signals
    flywheelLeadSupplyCurrentStatusSignal = flywheelLead.getSupplyCurrent();
    flywheelLeadStatorCurrentStatusSignal = flywheelLead.getStatorCurrent();
    flywheelLeadVelocityStatusSignal = flywheelLead.getVelocity();
    flywheelLeadTemperatureStatusSignal = flywheelLead.getDeviceTemp();
    flywheelLeadVoltageStatusSignal = flywheelLead.getMotorVoltage();
    flywheelLeadTorqueCurrentStatusSignal = flywheelLead.getTorqueCurrent();

    // Assign flywheel follow 1 status signals
    flywheelFollow1SupplyCurrentStatusSignal = flywheelFollow1.getSupplyCurrent();
    flywheelFollow1StatorCurrentStatusSignal = flywheelFollow1.getStatorCurrent();
    flywheelFollow1VelocityStatusSignal = flywheelFollow1.getVelocity();
    flywheelFollow1TemperatureStatusSignal = flywheelFollow1.getDeviceTemp();
    flywheelFollow1VoltageStatusSignal = flywheelFollow1.getMotorVoltage();
    flywheelFollow1TorqueCurrentStatusSignal = flywheelFollow1.getTorqueCurrent();

    // Assign flywheel follow 2 status signals
    flywheelFollow2SupplyCurrentStatusSignal = flywheelFollow2.getSupplyCurrent();
    flywheelFollow2StatorCurrentStatusSignal = flywheelFollow2.getStatorCurrent();
    flywheelFollow2VelocityStatusSignal = flywheelFollow2.getVelocity();
    flywheelFollow2TemperatureStatusSignal = flywheelFollow2.getDeviceTemp();
    flywheelFollow2VoltageStatusSignal = flywheelFollow2.getMotorVoltage();
    flywheelFollow2TorqueCurrentStatusSignal = flywheelFollow2.getTorqueCurrent();

    // Assign turret status signals
    turretStatorCurrentStatusSignal = turret.getStatorCurrent();
    turretSupplyCurrentStatusSignal = turret.getSupplyCurrent();
    turretTemperatureStatusSignal = turret.getDeviceTemp();
    turretVoltageStatusSignal = turret.getMotorVoltage();
    turretPositionStatusSignal = turret.getPosition();

    // Assign hood status signals
    hoodStatorCurrentStatusSignal = hood.getStatorCurrent();
    hoodSupplyCurrentStatusSignal = hood.getSupplyCurrent();
    hoodTemperatureStatusSignal = hood.getDeviceTemp();
    hoodVoltageStatusSignal = hood.getMotorVoltage();
    hoodPositionStatusSignal = hood.getPosition();

    Phoenix6Util.registerSignals(
        true,
        // FLYWHEEL LEAD
        flywheelLeadSupplyCurrentStatusSignal,
        flywheelLeadStatorCurrentStatusSignal,
        flywheelLeadVelocityStatusSignal,
        flywheelLeadTemperatureStatusSignal,
        flywheelLeadVoltageStatusSignal,
        flywheelLeadTorqueCurrentStatusSignal,

        // FLYWHEEL Follow 1
        flywheelFollow1SupplyCurrentStatusSignal,
        flywheelFollow1StatorCurrentStatusSignal,
        flywheelFollow1VelocityStatusSignal,
        flywheelFollow1TemperatureStatusSignal,
        flywheelFollow1VoltageStatusSignal,
        flywheelFollow1TorqueCurrentStatusSignal,

        // FLYWHEEL Follow 2
        flywheelFollow2SupplyCurrentStatusSignal,
        flywheelFollow2StatorCurrentStatusSignal,
        flywheelFollow2VelocityStatusSignal,
        flywheelFollow2TemperatureStatusSignal,
        flywheelFollow2VoltageStatusSignal,
        flywheelFollow2TorqueCurrentStatusSignal,

        // TURRET
        turretStatorCurrentStatusSignal,
        turretSupplyCurrentStatusSignal,
        turretTemperatureStatusSignal,
        turretVoltageStatusSignal,
        turretPositionStatusSignal,

        // HOOD
        hoodStatorCurrentStatusSignal,
        hoodSupplyCurrentStatusSignal,
        hoodTemperatureStatusSignal,
        hoodVoltageStatusSignal,
        hoodPositionStatusSignal);

    // Configure all motors
    configFlywheelLead(flywheelLead, "flywheel lead", flywheelLeadConfigAlert);
    configFlywheelFollow(flywheelFollow1, "flywheel follow 1", flywheelFollow1ConfigAlert);
    configFlywheelFollow(flywheelFollow2, "flywheel follow 2", flywheelFollow2ConfigAlert);
    configTurret(turret, TURRET_INVERTED, "turret", turretConfigAlert);
    configHood(hood, HOOD_INVERTED, "hood", hoodConfigAlert);

    this.flywheelSim =
        new FlywheelSystemSim(
            ShooterConstants.FLYWHEEL_KV,
            ShooterConstants.FLYWHEEL_KA,
            ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO,
            ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA,
            flywheelLead,
            flywheelFollow1,
            flywheelFollow2);
    this.turretLeadSim =
        new ArmSystemSim(
            turret,
            ShooterConstants.TURRET_INVERTED,
            ShooterConstants.TURRET_GEAR_RATIO,
            ShooterConstants.TURRET_LENGTH_METERS,
            ShooterConstants.TURRET_MASS_KG,
            ShooterConstants.TURRET_LOWER_ANGLE_LIMIT.in(Radians),
            ShooterConstants.TURRET_UPPER_ANGLE_LIMIT.in(Radians),
            ShooterConstants.TURRET_LOWER_ANGLE_LIMIT.in(Radians),
            false,
            ShooterConstants.SUBSYSTEM_NAME + " Turret");
    this.hoodLeadSim =
        new ArmSystemSim(
            hood,
            ShooterConstants.HOOD_INVERTED,
            ShooterConstants.HOOD_GEAR_RATIO,
            ShooterConstants.HOOD_LENGTH_METERS,
            ShooterConstants.HOOD_MASS_KG,
            ShooterConstants.HOOD_MIN_ANGLE.in(Radians),
            ShooterConstants.HOOD_MAX_ANGLE.in(Radians),
            ShooterConstants.HOOD_STARTING_ANGLE.in(Radians),
            false,
            ShooterConstants.SUBSYSTEM_NAME + " Hood");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelLeadConnected =
        flywheelLeadConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelLeadVelocityStatusSignal,
                flywheelLeadStatorCurrentStatusSignal,
                flywheelLeadSupplyCurrentStatusSignal,
                flywheelLeadTemperatureStatusSignal,
                flywheelLeadVoltageStatusSignal,
                flywheelLeadTorqueCurrentStatusSignal));
    inputs.flywheelFollow1Connected =
        flywheelFollow1ConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelFollow1StatorCurrentStatusSignal,
                flywheelFollow1SupplyCurrentStatusSignal,
                flywheelFollow1VelocityStatusSignal,
                flywheelFollow1TemperatureStatusSignal,
                flywheelFollow1VoltageStatusSignal,
                flywheelFollow1TorqueCurrentStatusSignal));
    inputs.flywheelFollow2Connected =
        flywheelFollow2ConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelFollow2StatorCurrentStatusSignal,
                flywheelFollow2SupplyCurrentStatusSignal,
                flywheelFollow2VelocityStatusSignal,
                flywheelFollow2TemperatureStatusSignal,
                flywheelFollow2VoltageStatusSignal,
                flywheelFollow2TorqueCurrentStatusSignal));
    inputs.hoodConnected =
        hoodConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                hoodStatorCurrentStatusSignal,
                hoodSupplyCurrentStatusSignal,
                hoodTemperatureStatusSignal,
                hoodVoltageStatusSignal,
                hoodPositionStatusSignal));
    inputs.turretConnected =
        turretConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                turretStatorCurrentStatusSignal,
                turretSupplyCurrentStatusSignal,
                turretTemperatureStatusSignal,
                turretVoltageStatusSignal,
                turretPositionStatusSignal));

    // Updates Flywheel Lead Motor Inputs
    inputs.flywheelLeadStatorCurrent = flywheelLeadStatorCurrentStatusSignal.getValue();
    inputs.flywheelLeadSupplyCurrent = flywheelLeadSupplyCurrentStatusSignal.getValue();
    inputs.flywheelLeadVelocity = flywheelLeadVelocityStatusSignal.getValue();
    inputs.flywheelLeadTemperature = flywheelLeadTemperatureStatusSignal.getValue();
    inputs.flywheelLeadVoltage = flywheelLeadVoltageStatusSignal.getValue();
    inputs.flywheelLeadTorqueCurrent = flywheelLeadTorqueCurrentStatusSignal.getValue();
    inputs.flywheelLeadReferenceVelocity = flywheelLeadReferenceVelocity.copy();

    // Updates Flywheel Follow1 Motor Inputs
    inputs.flywheelFollow1StatorCurrent = flywheelFollow1StatorCurrentStatusSignal.getValue();
    inputs.flywheelFollow1SupplyCurrent = flywheelFollow1SupplyCurrentStatusSignal.getValue();
    inputs.flywheelFollow1Velocity = flywheelFollow1VelocityStatusSignal.getValue();
    inputs.flywheelFollow1Temperature = flywheelFollow1TemperatureStatusSignal.getValue();
    inputs.flywheelFollow1Voltage = flywheelFollow1VoltageStatusSignal.getValue();
    inputs.flywheelFollow1TorqueCurrent = flywheelFollow1TorqueCurrentStatusSignal.getValue();

    // Updates Flywheel Follow2 Motor Inputs
    inputs.flywheelFollow2StatorCurrent = flywheelFollow2StatorCurrentStatusSignal.getValue();
    inputs.flywheelFollow2SupplyCurrent = flywheelFollow2SupplyCurrentStatusSignal.getValue();
    inputs.flywheelFollow2Velocity = flywheelFollow2VelocityStatusSignal.getValue();
    inputs.flywheelFollow2Temperature = flywheelFollow2TemperatureStatusSignal.getValue();
    inputs.flywheelFollow2Voltage = flywheelFollow2VoltageStatusSignal.getValue();
    inputs.flywheelFollow2TorqueCurrent = flywheelFollow2TorqueCurrentStatusSignal.getValue();

    // Updates Turret Motor Inputs
    inputs.turretStatorCurrent = turretStatorCurrentStatusSignal.getValue();
    inputs.turretSupplyCurrent = turretSupplyCurrentStatusSignal.getValue();
    inputs.turretTemperature = turretTemperatureStatusSignal.getValue();
    inputs.turretVoltage = turretVoltageStatusSignal.getValue();
    inputs.turretPosition = turretPositionStatusSignal.getValue();
    inputs.turretReferencePosition = this.turretReferencePosition;
    // Updates Hood Motor Inputs
    inputs.hoodStatorCurrent = hoodStatorCurrentStatusSignal.getValue();
    inputs.hoodSupplyCurrent = hoodSupplyCurrentStatusSignal.getValue();
    inputs.hoodTemperature = hoodTemperatureStatusSignal.getValue();
    inputs.hoodVoltage = hoodVoltageStatusSignal.getValue();
    inputs.hoodPosition = hoodPositionStatusSignal.getValue();
    inputs.hoodReferencePosition = this.hoodReferencePosition;

    if (Constants.TUNING_MODE) { // If the entire robot is in tuning mode
      // Flywheel Lead
      inputs.flywheelLeadClosedLoopReferenceVelocity =
          RotationsPerSecond.of(flywheelLead.getClosedLoopReference().getValue());
      inputs.flywheelLeadClosedLoopErrorVelocity =
          RotationsPerSecond.of(flywheelLead.getClosedLoopError().getValue());
      inputs.turretClosedLoopReferencePosition =
          Degrees.of(turret.getClosedLoopReference().getValue());
      inputs.turretClosedLoopErrorPosition = Degrees.of(turret.getClosedLoopError().getValue());
      inputs.hoodClosedLoopReferencePosition = Degrees.of(hood.getClosedLoopReference().getValue());
      inputs.hoodClosedLoopErrorPosition = Degrees.of(hood.getClosedLoopError().getValue());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          flywheelLead.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          config.kV = pid[4];
          config.kA = pid[5];
          flywheelLead.getConfigurator().apply(config);
        },
        flywheelLeadKP,
        flywheelLeadKI,
        flywheelLeadKD,
        flywheelLeadKS,
        flywheelLeadKV,
        flywheelLeadKA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();

          turret.getConfigurator().refresh(config);

          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          config.kV = pid[4];
          config.kA = pid[5];

          turret.getConfigurator().apply(config);
        },
        turretKP,
        turretKI,
        turretKD,
        turretKS,
        turretKV,
        turretKA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          hood.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          config.kV = pid[4];
          config.kA = pid[5];

          hood.getConfigurator().apply(config);
        },
        hoodKP,
        hoodKI,
        hoodKD,
        hoodKS,
        hoodKV,
        hoodKA);

    if (Constants.getMode() == Constants.Mode.SIM) { // If the entire robot is in simulation
      flywheelSim.updateSim();
      turretLeadSim.updateSim();
      hoodLeadSim.updateSim();
    }
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelLead.setControl(flywheelLeadVelocityRequest.withVelocity(velocity));

    this.flywheelLeadReferenceVelocity = velocity.copy();
  }

  @Override
  public void setFlywheelCurrent(Current amps) {
    flywheelLead.setControl(flywheelLeadCurrentRequest.withOutput(amps));
  }

  @Override
  public void setTurretPosition(Angle position) {
    turret.setControl(
        turretPositionRequest.withPosition(Units.degreesToRotations(position.in(Degrees))));
    this.turretReferencePosition = position.copy();
  }

  @Override
  public void setTurretVoltage(Voltage voltage) {
    turret.setControl(turretVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void setHoodPosition(Angle position) {
    hood.setControl(
        hoodPositionRequest.withPosition(Units.degreesToRotations(position.in(Degrees))));
    this.hoodReferencePosition = position.copy();
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hood.setControl(hoodVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void lowerHoodSlow(Voltage voltage) {
    hood.setControl(
        hoodVoltageRequest
            .withLimitReverseMotion(false)
            .withIgnoreSoftwareLimits(true)
            .withOutput(voltage));
  }

  @Override
  public void zeroHoodPosition() {
    hood.setControl(hoodVoltageRequest.withLimitReverseMotion(true).withOutput(0.0));
  }

  @Override
  public void zeroTurretPosition() {
    turret.setControl(turretVoltageRequest.withLimitReverseMotion(true).withOutput(0.0));
  }

  private void configFlywheelLead(TalonFX flywheelLead, String motorName, Alert configAlert) {

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.Slot0.kP = flywheelLeadKP.get();
    flywheelConfig.Slot0.kI = flywheelLeadKI.get();
    flywheelConfig.Slot0.kD = flywheelLeadKD.get();
    flywheelConfig.Slot0.kS = flywheelLeadKS.get();
    flywheelConfig.Slot0.kV = flywheelLeadKV.get();
    flywheelConfig.Slot0.kA = flywheelLeadKA.get();

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO;

    Phoenix6Util.applyAndCheckConfiguration(flywheelLead, flywheelConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, flywheelLead);
  }

  private void configFlywheelFollow(TalonFX flywheelFollow, String motorName, Alert configAlert) {

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO;

    Phoenix6Util.applyAndCheckConfiguration(flywheelFollow, flywheelConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, flywheelFollow);
  }

  private void configTurret(
      TalonFX turret, boolean isInverted, String motorName, Alert configAlert) {

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    turretConfig.CurrentLimits.SupplyCurrentLimit = TURRET_PEAK_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLowerLimit = TURRET_CONTINUOUS_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLowerTime = TURRET_PEAK_CURRENT_DURATION;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = TURRET_PEAK_CURRENT_LIMIT;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretConfig.Slot0.kP = turretKP.get();
    turretConfig.Slot0.kI = turretKI.get();
    turretConfig.Slot0.kD = turretKD.get();
    turretConfig.Slot0.kV = turretKV.get();
    turretConfig.Slot0.kA = turretKA.get();

    turretConfig.Feedback.SensorToMechanismRatio = ShooterConstants.TURRET_GEAR_RATIO;

    turretConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs turretLimitSwitches = turretConfig.SoftwareLimitSwitch;

    turretLimitSwitches.ForwardSoftLimitEnable = true;
    turretLimitSwitches.ForwardSoftLimitThreshold = TURRET_UPPER_ANGLE_LIMIT.in(Rotations);
    turretLimitSwitches.ReverseSoftLimitEnable = true;
    turretLimitSwitches.ReverseSoftLimitThreshold = TURRET_LOWER_ANGLE_LIMIT.in(Rotations);

    // Configure a hardware limit switch that zeros the turret position to its lower angle limit
    turretConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    turretConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
        TURRET_LOWER_ANGLE_LIMIT.in(Rotations);
    turretConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    Phoenix6Util.applyAndCheckConfiguration(turret, turretConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, turret);
  }

  private void configHood(TalonFX hood, boolean isInverted, String motorName, Alert configAlert) {

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.CurrentLimits.SupplyCurrentLimit = HOOD_PEAK_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = HOOD_CONTINUOUS_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLowerTime = HOOD_PEAK_CURRENT_DURATION;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HOOD_PEAK_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.Slot0.kP = hoodKP.get();
    hoodConfig.Slot0.kI = hoodKI.get();
    hoodConfig.Slot0.kD = hoodKD.get();
    hoodConfig.Slot0.kV = hoodKV.get();
    hoodConfig.Slot0.kA = hoodKA.get();
    hoodConfig.Slot0.kS = hoodKS.get();

    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;

    hoodConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs hoodLimitSwitches = hoodConfig.SoftwareLimitSwitch;

    hoodLimitSwitches.ForwardSoftLimitEnable = true;
    hoodLimitSwitches.ForwardSoftLimitThreshold = HOOD_UPPER_ANGLE_LIMIT.in(Rotations);
    hoodLimitSwitches.ReverseSoftLimitEnable = true;
    hoodLimitSwitches.ReverseSoftLimitThreshold = HOOD_LOWER_ANGLE_LIMIT.in(Rotations);

    // configure a hardware limit switch that zeros the elevator when lowered; there is no hardware
    // limit switch, but we will set it using a control request
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = HOOD_MIN_ANGLE.in(Rotations);
    hoodConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    Phoenix6Util.applyAndCheckConfiguration(hood, hoodConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, hood);
  }
}
