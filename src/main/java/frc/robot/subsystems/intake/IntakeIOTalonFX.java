package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX rollerMotor;
  private TalonFX deployerMotorLead;
  private TalonFX deployerMotorFollower;

  // Control requests
  private VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private TorqueCurrentFOC rollerCurrentRequest = new TorqueCurrentFOC(0);

  private PositionVoltage deployerPositionRequest = new PositionVoltage(0);
  private VoltageOut deployerVoltageRequest = new VoltageOut(0);
  private TorqueCurrentFOC deployerCurrentRequest = new TorqueCurrentFOC(0);

  // Alerts
  private final Alert rollerConfigAlert =
      new Alert("Failed to apply configuration for Intake Roller.", AlertType.kError);
  private final Alert deployerConfigAlert =
      new Alert("Failed to apply configuration for Intake Deployer.", AlertType.kError);
  private final Alert followerConfigAlert =
      new Alert("Failed to apply configuration for Intake Deployer Follower.", AlertType.kError);

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

  private StatusSignal<Voltage> deployerLeadVoltageSS;
  private StatusSignal<Angle> deployerLeadPositionSS;
  private StatusSignal<Current> deployerLeadStatorCurrentSS;
  private StatusSignal<Current> deployerLeadSupplyCurrentSS;
  private StatusSignal<Temperature> deployerLeadTempSS;

  private StatusSignal<Voltage> deployerFollowerVoltageSS;
  private StatusSignal<Current> deployerFollowerStatorCurrentSS;
  private StatusSignal<Current> deployerFollowerSupplyCurrentSS;
  private StatusSignal<Temperature> deployerFollowerTempSS;

  private double rollerReferenceVelocityRPS = 0.0;
  private double deployerReferencePositionRot = 0.0;

  private Debouncer connectedRollerDebouncer = new Debouncer(0.5);
  private Debouncer connectedDeployerDebouncer = new Debouncer(0.5);
  private Debouncer connectedDeployerFollowerDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    deployerMotorLead = new TalonFX(DEPLOYER_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    deployerMotorFollower =
        new TalonFX(DEPLOYER_FOLLOWER_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    // Initialize Signals
    rollerVoltageSS = rollerMotor.getMotorVoltage();
    rollerVelocitySS = rollerMotor.getVelocity();
    rollerStatorCurrentSS = rollerMotor.getStatorCurrent();
    rollerSupplyCurrentSS = rollerMotor.getSupplyCurrent();
    rollerTempSS = rollerMotor.getDeviceTemp();

    deployerLeadVoltageSS = deployerMotorLead.getMotorVoltage();
    deployerLeadPositionSS = deployerMotorLead.getPosition();
    deployerLeadStatorCurrentSS = deployerMotorLead.getStatorCurrent();
    deployerLeadSupplyCurrentSS = deployerMotorLead.getSupplyCurrent();
    deployerLeadTempSS = deployerMotorLead.getDeviceTemp();

    deployerFollowerVoltageSS = deployerMotorFollower.getMotorVoltage();
    deployerFollowerStatorCurrentSS = deployerMotorFollower.getStatorCurrent();
    deployerFollowerSupplyCurrentSS = deployerMotorFollower.getSupplyCurrent();
    deployerFollowerTempSS = deployerMotorFollower.getDeviceTemp();

    // Register with Phoenix6Util for optimized refreshing
    Phoenix6Util.registerSignals(
        true,
        rollerVoltageSS,
        rollerVelocitySS,
        rollerStatorCurrentSS,
        rollerSupplyCurrentSS,
        rollerTempSS,
        deployerLeadVoltageSS,
        deployerLeadPositionSS,
        deployerLeadStatorCurrentSS,
        deployerLeadSupplyCurrentSS,
        deployerLeadTempSS,
        deployerFollowerVoltageSS,
        deployerFollowerStatorCurrentSS,
        deployerFollowerSupplyCurrentSS,
        deployerFollowerTempSS);

    configDeployerMotorLead(deployerMotorLead);
    configDeployerMotorFollower(deployerMotorFollower);
    configRollerMotor(rollerMotor);

    deployerMotorFollower.setControl(
        new Follower(
            deployerMotorLead.getDeviceID(),
            MotorAlignmentValue
                .Opposed)); // FIXME: not sure what design is mechanically for motor alignment

    // Initialize Simulation
    this.rollerSim =
        new VelocitySystemSim(
            rollerMotor, ROLLER_MOTOR_INVERTED, ROLLER_KV, ROLLER_KA + 0.001, ROLLER_GEAR_RATIO);
    this.deployerSim =
        new ElevatorSystemSim(
            deployerMotorLead,
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
    inputs.deployerConnectedLead =
        connectedDeployerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                deployerLeadVoltageSS,
                deployerLeadStatorCurrentSS,
                deployerLeadSupplyCurrentSS,
                deployerLeadTempSS,
                deployerLeadPositionSS));

    inputs.deployerConnectedFollower =
        connectedDeployerFollowerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                deployerFollowerVoltageSS,
                deployerFollowerStatorCurrentSS,
                deployerFollowerSupplyCurrentSS,
                deployerFollowerTempSS));

    // Update Roller Inputs
    inputs.rollerVelocityRPS = rollerVelocitySS.getValueAsDouble();
    inputs.rollerStatorCurrent = rollerStatorCurrentSS.getValueAsDouble();
    inputs.rollerSupplyCurrent = rollerSupplyCurrentSS.getValueAsDouble();
    inputs.rollerTempCelsius = rollerTempSS.getValueAsDouble();
    inputs.rollerVoltage = rollerVoltageSS.getValueAsDouble();
    inputs.rollerReferenceVelocityRPS = this.rollerReferenceVelocityRPS;

    // Update Deployer Inputs
    inputs.deployerVoltageLead = deployerLeadVoltageSS.getValueAsDouble();
    inputs.deployerStatorCurrentLead = deployerLeadStatorCurrentSS.getValueAsDouble();
    inputs.deployerSupplyCurrentLead = deployerLeadSupplyCurrentSS.getValueAsDouble();
    inputs.deployerTempCelsiusLead = deployerLeadTempSS.getValueAsDouble();
    inputs.deployerAngularPositionRot = deployerLeadPositionSS.getValueAsDouble();
    inputs.deployerReferencePositionRot = this.deployerReferencePositionRot;

    inputs.deployerVoltageFollower = deployerFollowerVoltageSS.getValueAsDouble();
    inputs.deployerStatorCurrentFollower = deployerFollowerStatorCurrentSS.getValueAsDouble();
    inputs.deployerSupplyCurrentFollower = deployerFollowerSupplyCurrentSS.getValueAsDouble();
    inputs.deployerTempCelsiusFollower = deployerFollowerTempSS.getValueAsDouble();

    if (Constants.TUNING_MODE) {
      inputs.rollerClosedLoopErrorRPS = rollerMotor.getClosedLoopError().getValueAsDouble();
      inputs.rollerClosedLoopReferenceRPS = rollerMotor.getClosedLoopReference().getValueAsDouble();

      inputs.deployerClosedLoopErrorRot = deployerMotorLead.getClosedLoopError().getValueAsDouble();
      inputs.deployerClosedLoopReferenceRot =
          deployerMotorLead.getClosedLoopReference().getValueAsDouble();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.deployerMotorLead.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          this.deployerMotorLead.getConfigurator().apply(config);
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
    this.deployerMotorLead.setControl(deployerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setDeployerPosition(double angularPositionRot) {
    deployerMotorLead.setControl(deployerPositionRequest.withPosition(angularPositionRot));
    this.deployerReferencePositionRot = angularPositionRot;
  }

  @Override
  public void setDeployerCurrent(double amps) {
    this.deployerMotorLead.setControl(deployerCurrentRequest.withOutput(amps));
  }

  private void configDeployerMotorLead(TalonFX motor) {
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

  private void configDeployerMotorFollower(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = DEPLOYER_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = DEPLOYER_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    SoftwareLimitSwitchConfigs deployerLimitSwitches = config.SoftwareLimitSwitch;

    deployerLimitSwitches.ForwardSoftLimitEnable = true;
    deployerLimitSwitches.ForwardSoftLimitThreshold = DEPLOYER_MAX_ANGLE_ROT;
    deployerLimitSwitches.ReverseSoftLimitEnable = true;
    deployerLimitSwitches.ReverseSoftLimitThreshold = DEPLOYER_MIN_ANGLE_ROT;

    Phoenix6Util.applyAndCheckConfiguration(deployerMotorFollower, config, followerConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Deployer Motor Follower", motor);
  }

  private void configRollerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ROLLER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ROLLER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ROLLER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ROLLER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

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
