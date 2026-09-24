package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
import frc.lib.team3061.util.RobotOdometry;
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
  private StatusSignal<AngularVelocity> turretVelocityStatusSignal;

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

  private double hoodReferencePositionRot = 0.0;
  private double turretReferencePositionRot = 0.0;
  private double turretCurrentPositionRot = 0.0;
  private double flywheelLeadReferenceVelocityRPS = 0.0;

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
      new LoggedTunableNumber("Shooter/Flywheel kP", FLYWHEEL_KP);
  private final LoggedTunableNumber flywheelLeadKI =
      new LoggedTunableNumber("Shooter/Flywheel kI", FLYWHEEL_KI);
  private final LoggedTunableNumber flywheelLeadKD =
      new LoggedTunableNumber("Shooter/Flywheel kD", FLYWHEEL_KD);
  private final LoggedTunableNumber flywheelLeadKS =
      new LoggedTunableNumber("Shooter/Flywheel kS", FLYWHEEL_KS);
  private final LoggedTunableNumber flywheelLeadKV =
      new LoggedTunableNumber("Shooter/Flywheel kV", FLYWHEEL_KV);
  private final LoggedTunableNumber flywheelLeadKA =
      new LoggedTunableNumber("Shooter/Flywheel kA", FLYWHEEL_KA);
  private final LoggedTunableNumber turretFarKP =
      new LoggedTunableNumber("Shooter/Turret Far kP", TURRET_FAR_KP);
  private final LoggedTunableNumber turretCloseKP =
      new LoggedTunableNumber("Shooter/Turret Close kP", TURRET_CLOSE_KP);
  private final LoggedTunableNumber turretKI =
      new LoggedTunableNumber("Shooter/Turret kI", TURRET_KI);
  private final LoggedTunableNumber turretKD =
      new LoggedTunableNumber("Shooter/Turret kD", TURRET_KD);
  private final LoggedTunableNumber turretKS =
      new LoggedTunableNumber("Shooter/Turret kS", TURRET_KS);
  private final LoggedTunableNumber turretKV =
      new LoggedTunableNumber("Shooter/Turret kV", TURRET_KV);
  private final LoggedTunableNumber turretKA =
      new LoggedTunableNumber("Shooter/Turret kA", TURRET_KA);
  private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Shooter/Hood kP", HOOD_KP);
  private final LoggedTunableNumber hoodKI = new LoggedTunableNumber("Shooter/Hood kI", HOOD_KI);
  private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Shooter/Hood kD", HOOD_KD);
  private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Shooter/Hood kS", HOOD_KS);
  private final LoggedTunableNumber hoodKV = new LoggedTunableNumber("Shooter/Hood kV", HOOD_KV);
  private final LoggedTunableNumber hoodKA = new LoggedTunableNumber("Shooter/Hood kA", HOOD_KA);

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
            FLYWHEEL_FOLLOWER_1_INVERTED_FROM_LEAD
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned));
    // reversed
    flywheelFollow2.setControl(
        new Follower(
            FLYWHEEL_LEAD_MOTOR_ID,
            FLYWHEEL_FOLLOWER_2_INVERTED_FROM_LEAD
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
    flywheelLeadTorqueCurrentStatusSignal.setUpdateFrequency(1000.0);

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
    turretVelocityStatusSignal = turret.getVelocity();

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
        turretVelocityStatusSignal,

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
            FLYWHEEL_KV,
            FLYWHEEL_KA,
            FLYWHEEL_LEAD_GEAR_RATIO,
            FLYWHEEL_MOMENT_OF_INERTIA,
            flywheelLead,
            flywheelFollow1,
            flywheelFollow2);
    this.turretLeadSim =
        new ArmSystemSim(
            turret,
            TURRET_INVERTED,
            TURRET_GEAR_RATIO,
            TURRET_LENGTH_METERS,
            TURRET_MASS_KG,
            Units.rotationsToRadians(TURRET_LOWER_ANGLE_LIMIT_ROT),
            Units.rotationsToRadians(TURRET_UPPER_ANGLE_LIMIT_ROT),
            Units.rotationsToRadians(TURRET_STARTING_ANGLE_ROT),
            false,
            SUBSYSTEM_NAME + " Turret");
    this.hoodLeadSim =
        new ArmSystemSim(
            hood,
            HOOD_INVERTED,
            HOOD_GEAR_RATIO,
            HOOD_LENGTH_METERS,
            HOOD_MASS_KG,
            Units.rotationsToRadians(HOOD_MIN_ANGLE_ROT),
            Units.rotationsToRadians(HOOD_MAX_ANGLE_ROT),
            Units.rotationsToRadians(HOOD_STARTING_ANGLE_ROT),
            false,
            SUBSYSTEM_NAME + " Hood");
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
                turretPositionStatusSignal,
                turretVelocityStatusSignal));

    // Updates Flywheel Lead Motor Inputs
    inputs.flywheelLeadStatorCurrent = flywheelLeadStatorCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelLeadSupplyCurrent = flywheelLeadSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelLeadVelocityRPS = flywheelLeadVelocityStatusSignal.getValueAsDouble();
    inputs.flywheelLeadTemperature = flywheelLeadTemperatureStatusSignal.getValueAsDouble();
    inputs.flywheelLeadVoltage = flywheelLeadVoltageStatusSignal.getValueAsDouble();
    inputs.flywheelLeadTorqueCurrent = flywheelLeadTorqueCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelLeadReferenceVelocityRPS = flywheelLeadReferenceVelocityRPS;

    // Updates Flywheel Follow1 Motor Inputs
    inputs.flywheelFollow1StatorCurrent =
        flywheelFollow1StatorCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelFollow1SupplyCurrent =
        flywheelFollow1SupplyCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelFollow1VelocityRPS = flywheelFollow1VelocityStatusSignal.getValueAsDouble();
    inputs.flywheelFollow1Temperature = flywheelFollow1TemperatureStatusSignal.getValueAsDouble();
    inputs.flywheelFollow1Voltage = flywheelFollow1VoltageStatusSignal.getValueAsDouble();
    inputs.flywheelFollow1TorqueCurrent =
        flywheelFollow1TorqueCurrentStatusSignal.getValueAsDouble();

    // Updates Flywheel Follow2 Motor Inputs
    inputs.flywheelFollow2StatorCurrent =
        flywheelFollow2StatorCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelFollow2SupplyCurrent =
        flywheelFollow2SupplyCurrentStatusSignal.getValueAsDouble();
    inputs.flywheelFollow2VelocityRPS = flywheelFollow2VelocityStatusSignal.getValueAsDouble();
    inputs.flywheelFollow2Temperature = flywheelFollow2TemperatureStatusSignal.getValueAsDouble();
    inputs.flywheelFollow2Voltage = flywheelFollow2VoltageStatusSignal.getValueAsDouble();
    inputs.flywheelFollow2TorqueCurrent =
        flywheelFollow2TorqueCurrentStatusSignal.getValueAsDouble();

    // Updates Turret Motor Inputs
    inputs.turretStatorCurrent = turretStatorCurrentStatusSignal.getValueAsDouble();
    inputs.turretSupplyCurrent = turretSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.turretTemperature = turretTemperatureStatusSignal.getValueAsDouble();
    inputs.turretVoltage = turretVoltageStatusSignal.getValueAsDouble();
    inputs.turretPositionRot = turretPositionStatusSignal.getValueAsDouble();
    inputs.turretReferencePositionRot = this.turretReferencePositionRot;
    inputs.turretVelocityRPS = turretVelocityStatusSignal.getValueAsDouble();
    this.turretCurrentPositionRot = inputs.turretPositionRot;

    // Updates Hood Motor Inputs
    inputs.hoodStatorCurrent = hoodStatorCurrentStatusSignal.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.hoodTemperature = hoodTemperatureStatusSignal.getValueAsDouble();
    inputs.hoodVoltage = hoodVoltageStatusSignal.getValueAsDouble();
    inputs.hoodPositionRot = hoodPositionStatusSignal.getValueAsDouble();
    inputs.hoodReferencePositionRot = this.hoodReferencePositionRot;

    if (Constants.TUNING_MODE) { // If the entire robot is in tuning mode
      // Flywheel Lead
      inputs.flywheelLeadClosedLoopReferenceVelocityRPS =
          flywheelLead.getClosedLoopReference().getValue();
      inputs.flywheelLeadClosedLoopErrorVelocityRPS = flywheelLead.getClosedLoopError().getValue();
      inputs.turretClosedLoopReferencePositionRot = turret.getClosedLoopReference().getValue();
      inputs.turretClosedLoopErrorPositionRot = turret.getClosedLoopError().getValue();
      inputs.hoodClosedLoopReferencePositionRot = hood.getClosedLoopReference().getValue();
      inputs.hoodClosedLoopErrorPositionRot = hood.getClosedLoopError().getValue();
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

          Slot1Configs config1 = new Slot1Configs();

          turret.getConfigurator().refresh(config1);

          config1.kP = pid[6];
          config1.kI = pid[1];
          config1.kD = pid[2];
          config1.kS = pid[3];
          config1.kV = pid[4];
          config1.kA = pid[5];

          turret.getConfigurator().apply(config1);
        },
        turretCloseKP,
        turretKI,
        turretKD,
        turretKS,
        turretKV,
        turretKA,
        turretFarKP);

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
  public void setFlywheelVelocity(double velocityRPS) {
    flywheelLead.setControl(flywheelLeadVelocityRequest.withVelocity(velocityRPS));

    this.flywheelLeadReferenceVelocityRPS = velocityRPS;
  }

  @Override
  public void setFlywheelCurrent(double amps) {
    flywheelLead.setControl(flywheelLeadCurrentRequest.withOutput(amps));
  }

  @Override
  public void setTurretPosition(double positionRot) {
    turret.setControl(
        turretPositionRequest
            .withPosition(positionRot)
            .withVelocity(
                RadiansPerSecond.of(
                    -RobotOdometry.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond))
            .withSlot(
                Math.abs(this.turretReferencePositionRot - this.turretCurrentPositionRot)
                        < TURRET_CLOSE_POSITION_THRESHOLD_ROT
                    ? 0
                    : 1));
    this.turretReferencePositionRot = positionRot;
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turret.setControl(turretVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void setHoodPosition(double positionRot) {
    hood.setControl(hoodPositionRequest.withPosition(positionRot));
    this.hoodReferencePositionRot = positionRot;
  }

  @Override
  public void setHoodVoltage(double voltage) {
    hood.setControl(hoodVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void lowerHoodSlow(double voltage) {
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

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.Slot0.kP = flywheelLeadKP.get();
    flywheelConfig.Slot0.kI = flywheelLeadKI.get();
    flywheelConfig.Slot0.kD = flywheelLeadKD.get();
    flywheelConfig.Slot0.kS = flywheelLeadKS.get();
    flywheelConfig.Slot0.kV = flywheelLeadKV.get();
    flywheelConfig.Slot0.kA = flywheelLeadKA.get();

    flywheelConfig.Feedback.SensorToMechanismRatio = FLYWHEEL_LEAD_GEAR_RATIO;

    flywheelConfig.MotorOutput.Inverted =
        FLYWHEEL_LEAD_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(flywheelLead, flywheelConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, flywheelLead);
  }

  private void configFlywheelFollow(TalonFX flywheelFollow, String motorName, Alert configAlert) {

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = FLYWHEEL_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -FLYWHEEL_PEAK_CURRENT_LIMIT;

    flywheelConfig.Feedback.SensorToMechanismRatio = FLYWHEEL_LEAD_GEAR_RATIO;

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

    turretConfig.Slot0.kP = turretCloseKP.get();
    turretConfig.Slot0.kI = turretKI.get();
    turretConfig.Slot0.kD = turretKD.get();
    turretConfig.Slot0.kV = turretKV.get();
    turretConfig.Slot0.kA = turretKA.get();

    turretConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.00075;
    turretConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;
    turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    turretConfig.Slot1.kP = turretFarKP.get();
    turretConfig.Slot1.kI = turretKI.get();
    turretConfig.Slot1.kD = turretKD.get();
    turretConfig.Slot1.kV = turretKV.get();
    turretConfig.Slot1.kA = turretKA.get();

    turretConfig.Slot1.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;
    turretConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;

    turretConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs turretLimitSwitches = turretConfig.SoftwareLimitSwitch;

    turretLimitSwitches.ForwardSoftLimitEnable = true;
    turretLimitSwitches.ForwardSoftLimitThreshold = TURRET_UPPER_ANGLE_LIMIT_ROT;
    turretLimitSwitches.ReverseSoftLimitEnable = true;
    turretLimitSwitches.ReverseSoftLimitThreshold = TURRET_LOWER_ANGLE_LIMIT_ROT;

    // Configure a hardware limit switch that zeros the turret position to its lower angle limit
    turretConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    turretConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue =
        TURRET_LOWER_ANGLE_LIMIT_ROT;
    turretConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    Phoenix6Util.applyAndCheckConfiguration(turret, turretConfig, configAlert);

    turret.setPosition(TURRET_STARTING_ANGLE_ROT);

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
    hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    hoodConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.00003;
    hoodConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;

    hoodConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

    hoodConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SoftwareLimitSwitchConfigs hoodLimitSwitches = hoodConfig.SoftwareLimitSwitch;

    hoodLimitSwitches.ForwardSoftLimitEnable = true;
    hoodLimitSwitches.ForwardSoftLimitThreshold = HOOD_UPPER_ANGLE_LIMIT_ROT;
    hoodLimitSwitches.ReverseSoftLimitEnable = true;
    hoodLimitSwitches.ReverseSoftLimitThreshold = HOOD_LOWER_ANGLE_LIMIT_ROT;

    // configure a hardware limit switch that zeros the elevator when lowered; there is no hardware
    // limit switch, but we will set it using a control request
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    hoodConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = HOOD_MIN_ANGLE_ROT;
    hoodConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    Phoenix6Util.applyAndCheckConfiguration(hood, hoodConfig, configAlert);

    hood.setPosition(HOOD_STARTING_ANGLE_ROT);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, motorName, hood);
  }
}
