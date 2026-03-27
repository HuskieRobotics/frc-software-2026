package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX rollerMotor;
  private TalonFX deployerMotor;

  // Control requests
  private VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);
  private TorqueCurrentFOC rollerCurrentRequest = new TorqueCurrentFOC(0);

  private PositionVoltage deployerPositionRequest = new PositionVoltage(0);
  private VoltageOut deployerVoltageRequest = new VoltageOut(0);
  private TorqueCurrentFOC deployerCurrentRequest = new TorqueCurrentFOC(0);

  // Alerts
  private final Alert rollerConfigAlert =
      new Alert("Failed to apply configuration for Intake Roller.", AlertType.kError);
  private final Alert deployerConfigAlert =
      new Alert("Failed to apply configuration for Intake Deployer.", AlertType.kError);

  // Tunables
  private final LoggedTunableNumber rollerKp =
      new LoggedTunableNumber("Intake/Roller/kP", ROLLER_KP);
  private final LoggedTunableNumber rollerKi =
      new LoggedTunableNumber("Intake/Roller/kI", ROLLER_KI);
  private final LoggedTunableNumber rollerKd =
      new LoggedTunableNumber("Intake/Roller/kD", ROLLER_KD);

  private final LoggedTunableNumber rollerKs =
      new LoggedTunableNumber("Intake/Roller/kS", ROLLER_KS);
  private final LoggedTunableNumber rollerKv =
      new LoggedTunableNumber("Intake/Roller/kV", ROLLER_KV);

  private final LoggedTunableNumber deployerKp =
      new LoggedTunableNumber("Intake/Deployer/kP", DEPLOYER_KP);
  private final LoggedTunableNumber deployerKi =
      new LoggedTunableNumber("Intake/Deployer/kI", DEPLOYER_KI);
  private final LoggedTunableNumber deployerKd =
      new LoggedTunableNumber("Intake/Deployer/kD", DEPLOYER_KD);

  private final LoggedTunableNumber deployerKs =
      new LoggedTunableNumber("Intake/Deployer/kS", DEPLOYER_KS);

  private VelocitySystemSim rollerSim;
  private ElevatorSystemSim deployerSim;

  private StatusSignal<Voltage> rollerVoltageSS;
  private StatusSignal<AngularVelocity> rollerVelocitySS;
  private StatusSignal<Current> rollerStatorCurrentSS;
  private StatusSignal<Current> rollerSupplyCurrentSS;
  private StatusSignal<Temperature> rollerTempSS;

  private StatusSignal<Voltage> deployerVoltageSS;
  private StatusSignal<Angle> deployerPositionSS;
  private StatusSignal<Current> deployerStatorCurrentSS;
  private StatusSignal<Current> deployerSupplyCurrentSS;
  private StatusSignal<Temperature> deployerTempSS;

  private double rollerReferenceVelocityRPS = 0.0;
  private double deployerReferencePositionRot = 0.0;

  private Debouncer connectedRollerDebouncer = new Debouncer(0.5);
  private Debouncer connectedDeployerDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    deployerMotor = new TalonFX(DEPLOYER_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    // Initialize Signals
    rollerVoltageSS = rollerMotor.getMotorVoltage();
    rollerVelocitySS = rollerMotor.getVelocity();
    rollerStatorCurrentSS = rollerMotor.getStatorCurrent();
    rollerSupplyCurrentSS = rollerMotor.getSupplyCurrent();
    rollerTempSS = rollerMotor.getDeviceTemp();

    deployerVoltageSS = deployerMotor.getMotorVoltage();
    deployerPositionSS = deployerMotor.getPosition();
    deployerStatorCurrentSS = deployerMotor.getStatorCurrent();
    deployerSupplyCurrentSS = deployerMotor.getSupplyCurrent();
    deployerTempSS = deployerMotor.getDeviceTemp();

    // Register with Phoenix6Util for optimized refreshing
    Phoenix6Util.registerSignals(
        true,
        rollerVoltageSS,
        rollerVelocitySS,
        rollerStatorCurrentSS,
        rollerSupplyCurrentSS,
        rollerTempSS,
        deployerVoltageSS,
        deployerPositionSS,
        deployerStatorCurrentSS,
        deployerSupplyCurrentSS,
        deployerTempSS);

    configDeployerMotor(deployerMotor);
    configRollerMotor(rollerMotor);

    // Initialize Simulation
    this.rollerSim =
        new VelocitySystemSim(
            rollerMotor, ROLLER_MOTOR_INVERTED, ROLLER_KV, ROLLER_KA + 0.001, ROLLER_GEAR_RATIO);
    this.deployerSim =
        new ElevatorSystemSim(
            deployerMotor,
            DEPLOYER_MOTOR_INVERTED,
            DEPLOYER_GEAR_RATIO,
            DEPLOYER_MASS_KG,
            Units.inchesToMeters(0.5),
            RETRACTED_LINEAR_POSITION_METERS,
            DEPLOYED_LINEAR_POSITION_METERS,
            RETRACTED_LINEAR_POSITION_METERS,
            SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (Constants.getMode() == Constants.Mode.SIM) {
      if (deployerSim != null) {
        deployerSim.updateSim();
      }
      if (rollerSim != null) {
        rollerSim.updateSim();
      }
    }
    // Check connections
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

    // Update Roller Inputs
    inputs.rollerVelocityRPS = rollerVelocitySS.getValueAsDouble();
    inputs.rollerStatorCurrent = rollerStatorCurrentSS.getValueAsDouble();
    inputs.rollerSupplyCurrent = rollerSupplyCurrentSS.getValueAsDouble();
    inputs.rollerTempCelsius = rollerTempSS.getValueAsDouble();
    inputs.rollerVoltage = rollerVoltageSS.getValueAsDouble();
    inputs.rollerReferenceVelocityRPS = this.rollerReferenceVelocityRPS;

    // Update Deployer Inputs
    inputs.deployerVoltage = deployerVoltageSS.getValueAsDouble();
    inputs.deployerStatorCurrent = deployerStatorCurrentSS.getValueAsDouble();
    inputs.deployerSupplyCurrent = deployerSupplyCurrentSS.getValueAsDouble();
    inputs.deployerTempCelsius = deployerTempSS.getValueAsDouble();
    inputs.deployerAngularPositionRot = deployerPositionSS.getValueAsDouble();
    inputs.deployerReferencePositionRot = this.deployerReferencePositionRot;

    if (Constants.TUNING_MODE) {
      inputs.rollerClosedLoopErrorRPS = rollerMotor.getClosedLoopError().getValueAsDouble();
      inputs.rollerClosedLoopReferenceRPS = rollerMotor.getClosedLoopReference().getValueAsDouble();

      inputs.deployerClosedLoopErrorRot = deployerMotor.getClosedLoopError().getValueAsDouble();
      inputs.deployerClosedLoopReferenceRot =
          deployerMotor.getClosedLoopReference().getValueAsDouble();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.deployerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          this.deployerMotor.getConfigurator().apply(config);
        },
        deployerKp,
        deployerKi,
        deployerKd,
        deployerKs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.rollerMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          this.rollerMotor.getConfigurator().apply(config);
        },
        rollerKp,
        rollerKi,
        rollerKd,
        rollerKs,
        rollerKv);

    if (Constants.getMode() == Constants.Mode.SIM) {
      deployerSim.updateSim();
      rollerSim.updateSim();
    }
  }

  @Override
  public void setRollerVelocity(double velocityRPS) {
    this.rollerMotor.setControl(rollerVelocityRequest.withVelocity(velocityRPS));
    this.rollerReferenceVelocityRPS = velocityRPS;
  }

  @Override
  public void setRollerCurrent(double amps) {
    this.rollerMotor.setControl(rollerCurrentRequest.withOutput(amps));
  }

  @Override
  public void setDeployerVoltage(double voltage) {
    this.deployerMotor.setControl(deployerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setDeployerPosition(double angularPositionRot) {
    deployerMotor.setControl(deployerPositionRequest.withPosition(angularPositionRot));
    this.deployerReferencePositionRot = angularPositionRot;
  }

  @Override
  public void setDeployerCurrent(double amps) {
    this.deployerMotor.setControl(deployerCurrentRequest.withOutput(amps));
  }

  private void configDeployerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = DEPLOYER_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = DEPLOYER_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        DEPLOYER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    SoftwareLimitSwitchConfigs deployerLimitSwitches = config.SoftwareLimitSwitch;

    deployerLimitSwitches.ForwardSoftLimitEnable = true;
    deployerLimitSwitches.ForwardSoftLimitThreshold = DEPLOYER_MAX_ANGLE_ROT;
    deployerLimitSwitches.ReverseSoftLimitEnable = true;
    deployerLimitSwitches.ReverseSoftLimitThreshold = DEPLOYER_MIN_ANGLE_ROT;

    config.Feedback.SensorToMechanismRatio = DEPLOYER_GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = deployerKp.get();
    config.Slot0.kI = deployerKi.get();
    config.Slot0.kD = deployerKd.get();
    config.Slot0.kS = deployerKs.get();
    config.Slot0.kV = DEPLOYER_KV;
    config.Slot0.kA = DEPLOYER_KA;
    config.Slot0.kG = DEPLOYER_KG;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    Phoenix6Util.applyAndCheckConfiguration(motor, config, deployerConfigAlert);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Deployer Motor", motor);
  }

  private void configRollerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.TorqueCurrent.PeakForwardTorqueCurrent = ROLLER_PEAK_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ROLLER_PEAK_CURRENT_LIMIT;

    config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
    config.MotorOutput.Inverted =
        ROLLER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = rollerKp.get();
    config.Slot0.kI = rollerKi.get();
    config.Slot0.kD = rollerKd.get();
    config.Slot0.kS = rollerKs.get();
    config.Slot0.kV = rollerKv.get();
    config.Slot0.kA = ROLLER_KA;

    Phoenix6Util.applyAndCheckConfiguration(motor, config, rollerConfigAlert);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Roller Motor", motor);
  }
}
