package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.fuelIntake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rollerMotor;
  private TalonFX deployerMotor;

  // control output types
  private VelocityTorqueCurrentFOC rollerVelocityRequest;
  private TorqueCurrentFOC rollerCurrentRequest;
  private VoltageOut rollerVoltageRequest;
  private VoltageOut deployerVoltageRequest;
  private DynamicMotionMagicExpoVoltage deployerPositionRequest;

  private Alert rollerConfigAlert =
      new Alert("Failed to apply configuration for Intake Roller.", AlertType.kError);
  private Alert deployerConfigAlert =
      new Alert("Failed to apply configuration for Intake Deployer.", AlertType.kError);

  private final LoggedTunableNumber rollerKp =
      new LoggedTunableNumber("Intake/Roller/kP", IntakeConstants.ROLLER_KP);
  private final LoggedTunableNumber rollerKi =
      new LoggedTunableNumber("Intake/Roller/kI", IntakeConstants.ROLLER_KI);
  private final LoggedTunableNumber rollerKd =
      new LoggedTunableNumber("Intake/Roller/kD", IntakeConstants.ROLLER_KD);
  private final LoggedTunableNumber rollerKvExpo =
      new LoggedTunableNumber("Intake/Roller/kVExpo", IntakeConstants.ROLLER_KV_EXPO);
  private final LoggedTunableNumber rollerKaExpo =
      new LoggedTunableNumber("Intake/Roller/kAExpo", IntakeConstants.ROLLER_KA_EXPO);
  private final LoggedTunableNumber rollerCruiseVelocity =
      new LoggedTunableNumber(
          "Intake/Roller/cruiseVelocity", IntakeConstants.ROLLER_CRUISE_VELOCITY);

  private final LoggedTunableNumber deployerKp =
      new LoggedTunableNumber("Intake/Deployer/kP", IntakeConstants.DEPLOYER_KP);
  private final LoggedTunableNumber deployerKi =
      new LoggedTunableNumber("Intake/Deployer/kI", IntakeConstants.DEPLOYER_KI);
  private final LoggedTunableNumber deployerKd =
      new LoggedTunableNumber("Intake/Deployer/kD", IntakeConstants.DEPLOYER_KD);

  private final LoggedTunableNumber deployerKvExpo =
      new LoggedTunableNumber("Intake/Deployer/kVExpo", IntakeConstants.DEPLOYER_KV_EXPO);
  private final LoggedTunableNumber deployerKaExpo =
      new LoggedTunableNumber("Intake/Deployer/kAExpo", IntakeConstants.DEPLOYER_KA_EXPO);
  private final LoggedTunableNumber deployerCruiseVelocity =
      new LoggedTunableNumber(
          "Intake/Deployer/cruiseVelocity", IntakeConstants.DEPLOYER_CRUISE_VELOCITY);

  private VelocitySystemSim rollerSim;
  private ArmSystemSim deployerSim;

  // roller status signals
  private StatusSignal<Voltage> rollerVoltageSS;
  private StatusSignal<AngularVelocity> rollerVelocitySS;
  private StatusSignal<Current> rollerStatorCurrentSS;
  private StatusSignal<Current> rollerSupplyCurrentSS;
  private StatusSignal<Temperature> rollerTempSS;

  // deployer status signals
  private StatusSignal<Voltage> deployerVoltageSS;
  private StatusSignal<Current> deployerStatorCurrentSS;
  private StatusSignal<Current> deployerSupplyCurrentSS;
  private StatusSignal<Temperature> deployerTempSS;
  private StatusSignal<Angle> deployerPositionSS;

  private AngularVelocity rollerReferenceVelocity = RotationsPerSecond.of(0.0);
  private Angle deployerReferencePosition = Rotations.of(0);

  // debouncers
  private final Debouncer connectedRollerDebouncer = new Debouncer(0.5);
  private final Debouncer connectedDeployerDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    rollerMotor =
        new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    deployerMotor =
        new TalonFX(IntakeConstants.DEPLOYER_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    rollerVoltageSS = rollerMotor.getMotorVoltage();
    deployerVoltageSS = deployerMotor.getMotorVoltage();

    rollerVelocitySS = rollerMotor.getVelocity();

    rollerStatorCurrentSS = rollerMotor.getStatorCurrent();
    deployerStatorCurrentSS = deployerMotor.getStatorCurrent();

    rollerSupplyCurrentSS = rollerMotor.getSupplyCurrent();
    deployerSupplyCurrentSS = deployerMotor.getSupplyCurrent();

    rollerTempSS = rollerMotor.getDeviceTemp();
    deployerTempSS = deployerMotor.getDeviceTemp();

    deployerPositionSS = deployerMotor.getPosition();

    Phoenix6Util.registerSignals(
        true,
        rollerVoltageSS,
        deployerVoltageSS,
        rollerVelocitySS,
        rollerStatorCurrentSS,
        deployerStatorCurrentSS,
        rollerSupplyCurrentSS,
        deployerSupplyCurrentSS,
        rollerTempSS,
        deployerTempSS,
        deployerPositionSS);

    this.rollerSim =
        new VelocitySystemSim(
            rollerMotor, ROLLER_MOTOR_INVERTED, ROLLER_KV, ROLLER_KA, ROLLER_GEAR_RATIO);
    this.deployerSim =
        new ArmSystemSim(
            deployerMotor,
            DEPLOYER_MOTOR_INVERTED,
            DEPLOYER_GEAR_RATIO,
            DEPLOYER_LENGTH_METERS,
            DEPLOYER_MASS_KG,
            DEPLOYER_MIN_ANGLE,
            DEPLOYER_MAX_ANGLE,
            DEPLOYER_MIN_ANGLE,
            "Intake Deployer");

    rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    rollerCurrentRequest = new TorqueCurrentFOC(0);
    rollerVoltageRequest = new VoltageOut(0);
    deployerVoltageRequest = new VoltageOut(0);
    deployerPositionRequest =
        new DynamicMotionMagicExpoVoltage(0, deployerKvExpo.get(), deployerKaExpo.get());

    configDeployerMotor(deployerMotor);
    configRollerMotor(rollerMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerConnected =
        connectedRollerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                rollerVelocitySS,
                rollerVoltageSS,
                rollerStatorCurrentSS,
                rollerTempSS,
                rollerSupplyCurrentSS));
    inputs.deployerConnected =
        connectedDeployerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                deployerVoltageSS,
                deployerStatorCurrentSS,
                deployerSupplyCurrentSS,
                deployerTempSS,
                deployerPositionSS));

    inputs.rollerVelocity = rollerVelocitySS.getValue();
    inputs.rollerStatorCurrentAmps = rollerStatorCurrentSS.getValue();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentSS.getValue();
    inputs.rollerTempCelsius = rollerTempSS.getValue();
    inputs.rollerVoltage = rollerVoltageSS.getValue();
    inputs.rollerReferenceVelocityRadPerSec = this.rollerReferenceVelocity.copy();

    inputs.deployerVoltage = deployerVoltageSS.getValue();
    inputs.deployerStatorCurrentAmps = deployerStatorCurrentSS.getValue();
    inputs.deployerSupplyCurrentAmps = deployerSupplyCurrentSS.getValue();
    inputs.deployerTempCelsius = deployerTempSS.getValue();
    inputs.deployerReferencePositionDeg = this.deployerReferencePosition.copy();

    if (Constants.TUNING_MODE) {
      inputs.rollerClosedLoopError =
          Rotations.of(rollerMotor.getClosedLoopError().getValueAsDouble());
      inputs.deployerClosedLoopError =
          RotationsPerSecond.of(deployerMotor.getClosedLoopError().getValueAsDouble());

      inputs.rollerClosedLoopReference =
          Rotations.of(rollerMotor.getClosedLoopReference().getValueAsDouble());

      inputs.deployerClosedLoopReference =
          RotationsPerSecond.of(deployerMotor.getClosedLoopReference().getValueAsDouble());
    }

    inputs.angularPosition = deployerPositionSS.getValue();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.deployerMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = IntakeConstants.DEPLOYER_KS;
          config.Slot0.kV = IntakeConstants.DEPLOYER_KV;
          config.Slot0.kA = IntakeConstants.DEPLOYER_KA;
          config.Slot0.kG = IntakeConstants.DEPLOYER_KG;

          config.MotionMagic.MotionMagicExpo_kV = motionMagic[3];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[4];

          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[5];

          this.deployerMotor.getConfigurator().apply(config);
        },
        deployerKp,
        deployerKi,
        deployerKd,
        deployerKvExpo,
        deployerKaExpo,
        deployerCruiseVelocity);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.rollerMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = IntakeConstants.ROLLER_KS;
          config.Slot0.kV = IntakeConstants.ROLLER_KV;
          config.Slot0.kA = IntakeConstants.ROLLER_KA;
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[3];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[4];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[5];

          this.rollerMotor.getConfigurator().apply(config);
        },
        rollerKp,
        rollerKi,
        rollerKd,
        rollerKvExpo,
        rollerKaExpo,
        rollerCruiseVelocity);

    if (Constants.getMode() != Constants.Mode.SIM) {
      deployerSim.updateSim();
      rollerSim.updateSim();
    }
  }

  @Override
  public void setRollerVelocity(AngularVelocity velocity) {
    this.rollerMotor.setControl(rollerVelocityRequest.withVelocity(velocity));
    this.rollerReferenceVelocity = velocity.copy();
  }

  @Override
  public void setRollerCurrent(Current amps) {
    this.rollerMotor.setControl(rollerCurrentRequest.withOutput(amps));
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    this.rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setDeployerVoltage(Voltage voltage) {
    this.deployerMotor.setControl(deployerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setDeployerPosition(Angle angularPosition) {
    deployerMotor.setControl(deployerPositionRequest.withPosition(angularPosition.in(Rotations)));
    this.deployerReferencePosition = angularPosition.copy();
  }

  private void configDeployerMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.DEPLOYER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOYER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        DEPLOYER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.DEPLOYER_GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = deployerKp.get();
    config.Slot0.kI = deployerKi.get();
    config.Slot0.kD = deployerKd.get();
    config.Slot0.kS = IntakeConstants.DEPLOYER_KS;
    config.Slot0.kV = IntakeConstants.DEPLOYER_KV;
    config.Slot0.kA = IntakeConstants.DEPLOYER_KA;
    config.Slot0.kG = IntakeConstants.DEPLOYER_KG;
    config.MotionMagic.MotionMagicExpo_kV = deployerKvExpo.get();
    config.MotionMagic.MotionMagicExpo_kA = deployerKaExpo.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, deployerConfigAlert);

    FaultReporter.getInstance()
        .registerHardware(IntakeConstants.SUBSYSTEM_NAME, "Deployer Motor", motor);
  }

  private void configRollerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.ROLLER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -IntakeConstants.ROLLER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEAR_RATIO;
    config.MotorOutput.Inverted =
        ROLLER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = rollerKp.get();
    config.Slot0.kI = rollerKi.get();
    config.Slot0.kD = rollerKd.get();
    config.Slot0.kS = IntakeConstants.ROLLER_KS;
    config.Slot0.kV = IntakeConstants.ROLLER_KV;
    config.Slot0.kA = IntakeConstants.ROLLER_KA;
    config.MotionMagic.MotionMagicExpo_kV = rollerKvExpo.get();
    config.MotionMagic.MotionMagicExpo_kA = rollerKaExpo.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, rollerConfigAlert);

    FaultReporter.getInstance()
        .registerHardware(IntakeConstants.SUBSYSTEM_NAME, "Roller Motor", motor);
  }

  public void setDeployerHoldCurrentLimit(boolean isDeployed) {
    double limit =
        isDeployed
            ? IntakeConstants.DEPLOYER_HOLD_POSITION_CURRENT_LIMIT
            : IntakeConstants.DEPLOYER_CONTINUOUS_CURRENT_LIMIT;
    var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = limit;
    currentLimits.StatorCurrentLimitEnable = true;
    deployerMotor.getConfigurator().apply(currentLimits);
  }
}
