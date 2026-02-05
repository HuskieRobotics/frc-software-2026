package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants;

public class ShooterIOTalonFX implements ShooterIO {

  // We usually use VelocityTorqueCurrentFOC to control the velocity of a wheel.
  private VelocityTorqueCurrentFOC flywheelLeadVelocityRequest;
  private TorqueCurrentFOC flywheelLeadCurrentRequest;

  private DynamicMotionMagicExpoVoltage turretPositionRequest;
  private VoltageOut turretVoltageRequest;

  private DynamicMotionMagicExpoVoltage hoodPositionRequest;
  private VoltageOut hoodVoltageRequest;

  // Flywheel Lead Status Signals
  private StatusSignal<Current> flywheelLeadSupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelLeadStatorCurrentStatusSignal;

  // Flywheel Follow1 Status Signals
  private StatusSignal<Current> flywheelFollow1SupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow1StatorCurrentStatusSignal;

  // Flywheel Follow2 Status Signals
  private StatusSignal<Current> flywheelFollow2SupplyCurrentStatusSignal;
  private StatusSignal<Current> flywheelFollow2StatorCurrentStatusSignal;

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

  private Angle hoodMotorReferenceAngle = Rotations.of(0.0);
  private Angle turretMotorReferenceAngle = Rotations.of(0.0);

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

  // The following enables tuning of the PID and feedforward values by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.

  private final LoggedTunableNumber flywheelLeadKP =
      new LoggedTunableNumber("Shooter/Top kP", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KP);
  private final LoggedTunableNumber flywheelLeadKI =
      new LoggedTunableNumber("Shooter/Top kI", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KI);
  private final LoggedTunableNumber flywheelLeadKD =
      new LoggedTunableNumber("Shooter/Top kD", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KD);
  private final LoggedTunableNumber flywheelLeadKS =
      new LoggedTunableNumber("Shooter/Top kS", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KS);
  private final LoggedTunableNumber turretKP =
      new LoggedTunableNumber("Shooter/Turret kP", ShooterConstants.TURRET_ROTATION_KP);
  private final LoggedTunableNumber turretKI =
      new LoggedTunableNumber("Shooter/Turret kI", ShooterConstants.TURRET_ROTATION_KI);
  private final LoggedTunableNumber turretKD =
      new LoggedTunableNumber("Shooter/Turret kD", ShooterConstants.TURRET_ROTATION_KD);
  private final LoggedTunableNumber turretKV =
      new LoggedTunableNumber("Shooter/Turret kV", ShooterConstants.TURRET_ROTATION_KV);
  private final LoggedTunableNumber turretKA =
      new LoggedTunableNumber("Shooter/Turret kA", ShooterConstants.TURRET_ROTATION_KA);
  private final LoggedTunableNumber turretKVExpo =
      new LoggedTunableNumber("Shooter/Turret kVExpo", ShooterConstants.TURRET_ROTATION_EXPO_KV);
  private final LoggedTunableNumber turretKAExpo =
      new LoggedTunableNumber("Shooter/Turret kAExpo", ShooterConstants.TURRET_ROTATION_EXPO_KA);
  private final LoggedTunableNumber turretCruiseVelocity =
      new LoggedTunableNumber("Shooter/Turret Cruise Velocity", ShooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY);
  private final LoggedTunableNumber hoodKP =
      new LoggedTunableNumber("Shooter/Hood kP", ShooterConstants.HOOD_ROTATION_KP);
  private final LoggedTunableNumber hoodKI =
      new LoggedTunableNumber("Shooter/Hood kI", ShooterConstants.HOOD_ROTATION_KI);
  private final LoggedTunableNumber hoodKD =
      new LoggedTunableNumber("Shooter/Hood kD", ShooterConstants.HOOD_ROTATION_KD);
  private final LoggedTunableNumber hoodKS =
      new LoggedTunableNumber("Shooter/Hood kS", ShooterConstants.HOOD_ROTATION_KS);
  private final LoggedTunableNumber hoodKG =
      new LoggedTunableNumber("Shooter/Hood kG", ShooterConstants.HOOD_ROTATION_KG);
  private final LoggedTunableNumber hoodKV =
      new LoggedTunableNumber("Shooter/Hood kV", ShooterConstants.HOOD_ROTATION_KV);
    private final LoggedTunableNumber hoodKA =
      new LoggedTunableNumber("Shooter/Hood kA", ShooterConstants.HOOD_ROTATION_KA);
  private final LoggedTunableNumber hoodKVExpo =
      new LoggedTunableNumber("Shooter/Hood kVExpo", ShooterConstants.HOOD_ROTATION_EXPO_KV);
  private final LoggedTunableNumber hoodKAExpo =
      new LoggedTunableNumber("Shooter/Hood kAExpo", ShooterConstants.HOOD_ROTATION_EXPO_KA);
  private final LoggedTunableNumber hoodCruiseVelocity =
      new LoggedTunableNumber("Shooter/Hood Cruise Velocity", ShooterConstants.HOOD_MOTION_MAGIC_CRUISE_VELOCITY);

  // It is a bit more challenging to simulate a CANrange sensor compared to a DIO sensor. Using a
  // Tunable to simulate the distance to a game piece, requires that TUNING is set to true.
  private final LoggedTunableNumber simDetectorDistance =
      new LoggedTunableNumber("Shooter/Sim Detector Distance (m)", 1.0);

  private VelocitySystemSim flywheelLeadSim;
  private VelocitySystemSim flywheelFollow1Sim;
  private VelocitySystemSim flywheelFollow2Sim;
  private VelocitySystemSim turretLeadSim; // FIXME: determine correct sim type for turret
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
    hoodPositionRequest = new DynamicMotionMagicExpoVoltage(0, hoodKVExpo.get(), hoodKAExpo.get());

    turretVoltageRequest = new VoltageOut(0.0);
    turretPositionRequest = new DynamicMotionMagicExpoVoltage(0.0, turretKVExpo.get(), turretKAExpo.get());

    // Set up the flywheels 1 and 2 as followers of the lead flywheel
    flywheelFollow1.setControl(
        new Follower(
            FLYWHEEL_LEAD_MOTOR_ID,
            MotorAlignmentValue.Aligned)); // FIXME: change from aligned to opposed if reversed
    flywheelFollow2.setControl(
        new Follower(
            FLYWHEEL_LEAD_MOTOR_ID,
            MotorAlignmentValue.Aligned)); // FIXME: change from aligned to opposed if reversed

    // Assign FLYWHEEL LEAD status signals
    flywheelLeadSupplyCurrentStatusSignal = flywheelLead.getSupplyCurrent();
    flywheelLeadStatorCurrentStatusSignal = flywheelLead.getStatorCurrent();
    flywheelLeadVelocityStatusSignal = flywheelLead.getVelocity();
    flywheelLeadTemperatureStatusSignal = flywheelLead.getDeviceTemp();
    flywheelLeadVoltageStatusSignal = flywheelLead.getMotorVoltage();

    // Assign flywheel follow 1 status signals
    flywheelFollow1SupplyCurrentStatusSignal = flywheelFollow1.getSupplyCurrent();
    flywheelFollow1StatorCurrentStatusSignal = flywheelFollow1.getStatorCurrent();
    flywheelFollow1VelocityStatusSignal = flywheelFollow1.getVelocity();
    flywheelFollow1TemperatureStatusSignal = flywheelFollow1.getDeviceTemp();
    flywheelFollow1VoltageStatusSignal = flywheelFollow1.getMotorVoltage();

    // Assign flywheel follow 2 status signals
    flywheelFollow2SupplyCurrentStatusSignal = flywheelFollow2.getSupplyCurrent();
    flywheelFollow2StatorCurrentStatusSignal = flywheelFollow2.getStatorCurrent();
    flywheelFollow2VelocityStatusSignal = flywheelFollow2.getVelocity();
    flywheelFollow2TemperatureStatusSignal = flywheelFollow2.getDeviceTemp();
    flywheelFollow2VoltageStatusSignal = flywheelFollow2.getMotorVoltage();

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

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        // FLYWHEEL LEAD
        flywheelLeadSupplyCurrentStatusSignal,
        flywheelLeadStatorCurrentStatusSignal,
        flywheelLeadVelocityStatusSignal,
        flywheelLeadTemperatureStatusSignal,
        flywheelLeadVoltageStatusSignal,

        // FLYWHEEL Follow 1
        flywheelFollow1SupplyCurrentStatusSignal,
        flywheelFollow1StatorCurrentStatusSignal,
        flywheelFollow1VelocityStatusSignal,
        flywheelFollow1TemperatureStatusSignal,
        flywheelFollow1VoltageStatusSignal,

        // FLYWHEEL Follow 2
        flywheelFollow2SupplyCurrentStatusSignal,
        flywheelFollow2StatorCurrentStatusSignal,
        flywheelFollow2VelocityStatusSignal,
        flywheelFollow2TemperatureStatusSignal,
        flywheelFollow2VoltageStatusSignal,

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
    configFlywheel(
        flywheelLead, FLYWHEEL_LEAD_INVERTED, "flywheel lead", true, flywheelLeadConfigAlert);
    configFlywheel(
        flywheelFollow1,
        FLYWHEEL_FOLLOW_1_INVERTED,
        "flywheel follow 1",
        false,
        flywheelFollow1ConfigAlert);
    configFlywheel(
        flywheelFollow2,
        FLYWHEEL_FOLLOW_2_INVERTED,
        "flywheel follow 2",
        false,
        flywheelFollow2ConfigAlert);
    configTurret(turret, TURRET_INVERTED, "turret", turretConfigAlert);
    configHood(hood, HOOD_INVERTED, "hood", hoodConfigAlert);

    // Create a simulation objects for the shooter. The specific parameters for the simulation
    // are determined based on the mechanical design of the shooter.
    this.flywheelLeadSim =
        new VelocitySystemSim(
            flywheelLead,
            ShooterConstants.FLYWHEEL_LEAD_INVERTED,
            ShooterConstants.SIM_KV, // update as needed
            ShooterConstants.SIM_KA, // update as needed
            ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO);
    this.turretLeadSim =
        new VelocitySystemSim(
            turret,
            ShooterConstants.TURRET_INVERTED,
            ShooterConstants.SIM_KV,
            ShooterConstants.SIM_KA,
            ShooterConstants.TURRET_GEAR_RATIO);
    this.hoodLeadSim =
        new ArmSystemSim(
            hood,
            ShooterConstants.HOOD_INVERTED,
            ShooterConstants.HOOD_GEAR_RATIO,
            ShooterConstants.HOOD_LENGTH_METERS,
            ShooterConstants.HOOD_MASS_KG,
            ShooterConstants.HOOD_MIN_ANGLE_RAD,
            ShooterConstants.HOOD_MAX_ANGLE_RAD,
            ShooterConstants.HOOD_STARTING_ANGLE_RAD,
            ShooterConstants.SUBSYSTEM_NAME + " Hood");
    this.flywheelFollow1Sim =
        new VelocitySystemSim(
            flywheelFollow1,
            ShooterConstants.FLYWHEEL_FOLLOW_1_INVERTED,
            ShooterConstants.SIM_KV,
            ShooterConstants.SIM_KA,
            ShooterConstants.FLYWHEEL_FOLLOW_1_GEAR_RATIO);
    this.flywheelFollow2Sim =
        new VelocitySystemSim(
            flywheelFollow2,
            ShooterConstants.FLYWHEEL_FOLLOW_2_INVERTED,
            ShooterConstants.SIM_KV,
            ShooterConstants.SIM_KA,
            ShooterConstants.FLYWHEEL_FOLLOW_2_GEAR_RATIO);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Determine if the motors for the shooter are still connected (i.e., reachable on the CAN bus).
    // We do this by verifying that none of the status signals for the device report an error.
    inputs.flywheelLeadConnected =
        flywheelLeadConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelLeadVelocityStatusSignal,
                flywheelLeadStatorCurrentStatusSignal,
                flywheelLeadSupplyCurrentStatusSignal,
                flywheelLeadTemperatureStatusSignal,
                flywheelLeadVoltageStatusSignal));
    inputs.flywheelFollow1Connected =
        flywheelFollow1ConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelFollow1StatorCurrentStatusSignal,
                flywheelFollow1SupplyCurrentStatusSignal,
                flywheelFollow1VelocityStatusSignal,
                flywheelFollow1TemperatureStatusSignal,
                flywheelFollow1VoltageStatusSignal));
    inputs.flywheelFollow2Connected =
        flywheelFollow2ConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                flywheelFollow2StatorCurrentStatusSignal,
                flywheelFollow2SupplyCurrentStatusSignal,
                flywheelFollow2VelocityStatusSignal,
                flywheelFollow2TemperatureStatusSignal,
                flywheelFollow2VoltageStatusSignal));
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

    // Updates Flywheel Follow1 Motor Inputs
    inputs.flywheelFollow1StatorCurrent = flywheelFollow1StatorCurrentStatusSignal.getValue();
    inputs.flywheelFollow1SupplyCurrent = flywheelFollow1SupplyCurrentStatusSignal.getValue();
    inputs.flywheelFollow1Velocity = flywheelFollow1VelocityStatusSignal.getValue();
    inputs.flywheelFollow1Temperature = flywheelFollow1TemperatureStatusSignal.getValue();
    inputs.flywheelFollow1Voltage = flywheelFollow1VoltageStatusSignal.getValue();

    // Updates Flywheel Follow2 Motor Inputs
    inputs.flywheelFollow2StatorCurrent = flywheelFollow2StatorCurrentStatusSignal.getValue();
    inputs.flywheelFollow2SupplyCurrent = flywheelFollow2SupplyCurrentStatusSignal.getValue();
    inputs.flywheelFollow2Velocity = flywheelFollow2VelocityStatusSignal.getValue();
    inputs.flywheelFollow2Temperature = flywheelFollow2TemperatureStatusSignal.getValue();
    inputs.flywheelFollow2Voltage = flywheelFollow2VoltageStatusSignal.getValue();

    // Updates Turret Motor Inputs
    inputs.turretStatorCurrent = turretStatorCurrentStatusSignal.getValue();
    inputs.turretSupplyCurrent = turretSupplyCurrentStatusSignal.getValue();
    inputs.turretTemperature = turretTemperatureStatusSignal.getValue();
    inputs.turretVoltage = turretVoltageStatusSignal.getValue();
    inputs.turretPosition = turretPositionStatusSignal.getValue();
    inputs.turretReferencePosition = this.turretMotorReferenceAngle;


    // Updates Hood Motor Inputs
    inputs.hoodStatorCurrent = hoodStatorCurrentStatusSignal.getValue();
    inputs.hoodSupplyCurrent = hoodSupplyCurrentStatusSignal.getValue();
    inputs.hoodTemperature = hoodTemperatureStatusSignal.getValue();
    inputs.hoodVoltage = hoodVoltageStatusSignal.getValue();
    inputs.hoodPosition = hoodPositionStatusSignal.getValue();
    inputs.hoodReferencePosition = this.hoodMotorReferenceAngle;

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem. For example, the
    // shootMotorTopReferenceVelocityRPS property should be used throughout the subsystem since it
    // will always be populated.

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

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          flywheelLead.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          flywheelLead.getConfigurator().apply(config);
        },
        flywheelLeadKP,
        flywheelLeadKI,
        flywheelLeadKD,
        flywheelLeadKS);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {

            TalonFXConfiguration config = new TalonFXConfiguration();

          turret.getConfigurator().refresh(config);

          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kV = motionMagic[3];
          config.Slot0.kA = motionMagic[4];
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[5];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[6];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[7];

          turret.getConfigurator().apply(config);
        },
        turretKP,
        turretKI,
        turretKD,
        turretKV,
        turretKA,
        turretKVExpo,
        turretKAExpo,
        turretCruiseVelocity);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          hood.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];

          hood.getConfigurator().apply(config);
        },
        hoodKP,
        hoodKI,
        hoodKD,
        hoodKS,
        hoodKV,
        hoodKA,
        hoodKG,
        hoodKVExpo,
        hoodKAExpo,
        hoodCruiseVelocity);

    // The last step in the updateInputs method is to update the simulation.
    if (Constants.getMode() == Constants.Mode.SIM) { // If the entire robot is in simulation
      flywheelLeadSim.updateSim();
      turretLeadSim.updateSim();
      hoodLeadSim.updateSim();
      flywheelFollow1Sim.updateSim();
      flywheelFollow2Sim.updateSim();
    }
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelLead.setControl(
        flywheelLeadVelocityRequest.withVelocity(
            velocity)); // velocities for followers should be set automatically
    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
  }

  @Override
  public void setFlywheelCurrent(Current amps) {
    flywheelLead.setControl(flywheelLeadCurrentRequest.withOutput(amps));
  }

  @Override
  public void setTurretPosition(Angle position) {
    turret.setControl(turretPositionRequest.withPosition(position.in(Rotations)));
    this.turretMotorReferenceAngle = position;
  }

  @Override
  public void setTurretVoltage(Voltage voltage) {
    turret.setControl(turretVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setHoodPosition(Angle angle) {
    hood.setControl(hoodPositionRequest.withPosition(angle.in(Rotations)));
    this.hoodMotorReferenceAngle = angle;
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hood.setControl(hoodVoltageRequest.withOutput(voltage));
  }

  private void configFlywheel(
      TalonFX flywheel, boolean isInverted, String motorName, boolean isLead, Alert configAlert) {

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.FLYWHEEL_PEAK_CURRENT_LIMIT;
    if (isLead) {
      flywheelConfig.Slot0.kP = flywheelLeadKP.get();
      flywheelConfig.Slot0.kI = flywheelLeadKI.get();
      flywheelConfig.Slot0.kD = flywheelLeadKD.get();
      flywheelConfig.Slot0.kS = flywheelLeadKS.get();

      flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO;
    }

    flywheelConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(flywheel, flywheelConfig, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, flywheel);
  }

  private void configTurret(
      TalonFX turret, boolean isInverted, String motorName, Alert configAlert) {

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    turretConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.TURRET_PEAK_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLowerLimit =
        ShooterConstants.TURRET_CONTINUOUS_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLowerTime =
        ShooterConstants.TURRET_PEAK_CURRENT_DURATION;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.TURRET_PEAK_CURRENT_LIMIT;

    turretConfig.Slot0.kP = turretKP.get();
    turretConfig.Slot0.kI = turretKI.get();
    turretConfig.Slot0.kD = turretKD.get();
    turretConfig.Slot0.kV = turretKV.get();
    turretConfig.Slot0.kA = turretKA.get();
    
    turretConfig.MotionMagic.MotionMagicExpo_kA = turretKAExpo.get();
    turretConfig.MotionMagic.MotionMagicExpo_kV = turretKVExpo.get(); 
    turretConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

    turretConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY;
    
    turretConfig.Feedback.SensorToMechanismRatio = ShooterConstants.TURRET_GEAR_RATIO;

    turretConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs turretLimitSwitches = turretConfig.SoftwareLimitSwitch;

    turretLimitSwitches.ForwardSoftLimitEnable = true;
    turretLimitSwitches.ForwardSoftLimitThreshold = TURRET_UPPER_ANGLE_LIMIT.in(Rotations);
    turretLimitSwitches.ReverseSoftLimitEnable = true;
    turretLimitSwitches.ReverseSoftLimitThreshold = TURRET_LOWER_ANGLE_LIMIT.in(Rotations);

    Phoenix6Util.applyAndCheckConfiguration(turret, turretConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, turret);
  }

  private void configHood(TalonFX hood, boolean isInverted, String motorName, Alert configAlert) {

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.HOOD_PEAK_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLowerLimit =
        ShooterConstants.HOOD_CONTINUOUS_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.HOOD_PEAK_CURRENT_DURATION;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ShooterConstants.HOOD_PEAK_CURRENT_LIMIT;

    hoodConfig.Slot0.kP = hoodKP.get();
    hoodConfig.Slot0.kI = hoodKI.get();
    hoodConfig.Slot0.kD = hoodKD.get();
    hoodConfig.Slot0.kV = hoodKV.get();
    hoodConfig.Slot0.kA = hoodKA.get();
    hoodConfig.MotionMagic.MotionMagicExpo_kV = hoodKVExpo.get();
    hoodConfig.MotionMagic.MotionMagicExpo_kA = hoodKAExpo.get();
    hoodConfig.Slot0.kS = hoodKS.get();
    hoodConfig.Slot0.kG = hoodKG.get();
    hoodConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.HOOD_MOTION_MAGIC_CRUISE_VELOCITY;

    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO;

    hoodConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs hoodLimitSwitches = hoodConfig.SoftwareLimitSwitch;

    hoodLimitSwitches.ForwardSoftLimitEnable = true;
    hoodLimitSwitches.ForwardSoftLimitThreshold = HOOD_UPPER_ANGLE_LIMIT.in(Rotations);
    hoodLimitSwitches.ReverseSoftLimitEnable = true;
    hoodLimitSwitches.ReverseSoftLimitThreshold = HOOD_LOWER_ANGLE_LIMIT.in(Rotations);

    Phoenix6Util.applyAndCheckConfiguration(hood, hoodConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, hood);
  }
}
