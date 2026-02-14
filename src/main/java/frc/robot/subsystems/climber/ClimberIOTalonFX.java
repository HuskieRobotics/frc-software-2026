package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {

  private VoltageOut climberVoltageRequest;
  private MotionMagicExpoVoltage climberPositionRequest;

  private StatusSignal<Voltage> climberMotorVoltageStatusSignal;
  private StatusSignal<Current> climberMotorStatorCurrentStatusSignal;
  private StatusSignal<Current> climberMotorSupplyCurrentStatusSignal;
  private StatusSignal<Temperature> climberMotorTemperatureStatusSignal;
  private StatusSignal<Angle> climberMotorPositionRotationsStatusSignal;

  private Angle climberReferenceAngle = Rotations.of(0.0);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private ArmSystemSim climberSim;

  private final TalonFX climberMotor;
  private final CANcoder angleEncoder;

  private Alert configAlert =
      new Alert("Failed to apply configuration for Climber.", AlertType.kError);

  // climber tunable numbers

  private final LoggedTunableNumber climberMotorKP =
      new LoggedTunableNumber("Climber/MotorKP", ClimberConstants.CLIMBER_KP);

  private final LoggedTunableNumber climberMotorKI =
      new LoggedTunableNumber("Climber/MotorKI", ClimberConstants.CLIMBER_KI);

  private final LoggedTunableNumber climberMotorKD =
      new LoggedTunableNumber("Climber/MotorKD", ClimberConstants.CLIMBER_KD);

  private final LoggedTunableNumber climberMotorKS =
      new LoggedTunableNumber("Climber/MotorKS", ClimberConstants.CLIMBER_KS);

  private final LoggedTunableNumber climberMotorKV =
      new LoggedTunableNumber("Climber/MotorKV", ClimberConstants.CLIMBER_KV);

  private final LoggedTunableNumber climberMotorKA =
      new LoggedTunableNumber("Climber/MotorKA", ClimberConstants.CLIMBER_KA);

  private final LoggedTunableNumber climberMotorKG =
      new LoggedTunableNumber("Climber/MotorKG", ClimberConstants.CLIMBER_KG);

  private final LoggedTunableNumber climberMotorKVExpo =
      new LoggedTunableNumber("Climber/MotorKVExpo", ClimberConstants.CLIMBER_KV_EXPO);

  private final LoggedTunableNumber climberMotorKAExpo =
      new LoggedTunableNumber("Climber/MotorKAExpo", ClimberConstants.CLIMBER_KA_EXPO);

  private final LoggedTunableNumber climberCruiseVelocity =
      new LoggedTunableNumber(
          "Climber/MotorCruiseVelocity", ClimberConstants.CLIMBER_CRUISE_VELOCITY);

  // Cancoder tunable numbers
  private final LoggedTunableNumber rotationEncoderMagnetOffset =
      new LoggedTunableNumber(
          "Climber/ROTATION_MAGNET_OFFSET", ClimberConstants.CLIMBER_MAGNET_OFFSET);

  public ClimberIOTalonFX() {

    climberMotor = new TalonFX(CLIMBER_CAN_ID, RobotConfig.getInstance().getCANBus());
    angleEncoder = new CANcoder(ANGLE_ENCODER_ID, RobotConfig.getInstance().getCANBus());

    climberVoltageRequest = new VoltageOut(0.0);
    climberPositionRequest = new MotionMagicExpoVoltage(0.0);

    climberMotorVoltageStatusSignal = climberMotor.getMotorVoltage();
    climberMotorStatorCurrentStatusSignal = climberMotor.getStatorCurrent();
    climberMotorSupplyCurrentStatusSignal = climberMotor.getSupplyCurrent();
    climberMotorTemperatureStatusSignal = climberMotor.getDeviceTemp();
    climberMotorPositionRotationsStatusSignal = climberMotor.getPosition();

    Phoenix6Util.registerSignals(
        true,
        climberMotorVoltageStatusSignal,
        climberMotorStatorCurrentStatusSignal,
        climberMotorSupplyCurrentStatusSignal,
        climberMotorTemperatureStatusSignal,
        climberMotorPositionRotationsStatusSignal);

    configClimberMotor(climberMotor, angleEncoder);

    climberSim =
        new ArmSystemSim(
            climberMotor,
            angleEncoder,
            CLIMBER_MOTOR_INVERTED,
            ENCODER_GEAR_RATIO,
            CLIMBER_GEAR_RATIO,
            CLIMBER_LENGTH_INCHES,
            CLIMBER_MASS_KG,
            MIN_ANGLE_DEGREES.in(Degrees),
            MAX_ANGLE_DEGREES.in(Degrees),
            MIN_ANGLE_DEGREES.in(Degrees),
            SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                climberMotorVoltageStatusSignal,
                climberMotorStatorCurrentStatusSignal,
                climberMotorSupplyCurrentStatusSignal,
                climberMotorTemperatureStatusSignal,
                climberMotorPositionRotationsStatusSignal));

    inputs.voltageSupplied = climberMotorVoltageStatusSignal.getValue();
    inputs.statorCurrent = climberMotorStatorCurrentStatusSignal.getValue();
    inputs.supplyCurrent = climberMotorSupplyCurrentStatusSignal.getValue();
    inputs.motorTemperature = climberMotorTemperatureStatusSignal.getValue();
    inputs.climberAngle = climberMotorPositionRotationsStatusSignal.getValue();

    inputs.referenceAngle = this.climberReferenceAngle;

    if (Constants.TUNING_MODE) {

      inputs.closedLoopError = Rotations.of(climberMotor.getClosedLoopError().getValue());
      inputs.closedLoopReference = Rotations.of(climberMotor.getClosedLoopReference().getValue());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.climberMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          config.Slot0.kS = pid[3];
          config.Slot0.kV = pid[4];
          config.Slot0.kA = pid[5];
          config.Slot0.kG = pid[6];
          config.MotionMagic.MotionMagicExpo_kV = pid[7];
          config.MotionMagic.MotionMagicExpo_kA = pid[8];
          config.MotionMagic.MotionMagicCruiseVelocity = pid[9];
          this.climberMotor.getConfigurator().apply(config);
        },
        climberMotorKP,
        climberMotorKI,
        climberMotorKD,
        climberMotorKS,
        climberMotorKV,
        climberMotorKA,
        climberMotorKG,
        climberMotorKVExpo,
        climberMotorKAExpo,
        climberCruiseVelocity);

    // CANcoder configuration update
    if (rotationEncoderMagnetOffset.hasChanged(hashCode())) {
      CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
      angleEncoder.getConfigurator().refresh(angleCANCoderConfig);
      angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
      angleEncoder.getConfigurator().apply(angleCANCoderConfig);
    }

    climberSim.updateSim();
  }

  @Override
  public void setClimberVoltage(Voltage voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    climberMotor.setControl(climberVoltageRequest.withLimitReverseMotion(true).withOutput(0));
  }

  @Override
  public void setClimberAngle(Angle angle) {
    this.climberReferenceAngle = angle;
    climberMotor.setControl(climberPositionRequest.withPosition(angle.in(Rotations)));
  }

  private void configClimberMotor(TalonFX motor, CANcoder angleEncoder) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // CANcoder configuration
    CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
    angleCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    angleCANCoderConfig.MagnetSensor.SensorDirection =
        CLIMBER_ENCODER_INVERTED
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
    Phoenix6Util.applyAndCheckConfiguration(angleEncoder, angleCANCoderConfig, configAlert);

    config.CurrentLimits.SupplyCurrentLowerLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.StatorCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = climberMotorKP.get();
    config.Slot0.kI = climberMotorKI.get();
    config.Slot0.kD = climberMotorKD.get();
    config.Slot0.kS = climberMotorKS.get();
    config.Slot0.kV = climberMotorKV.get();
    config.Slot0.kA = climberMotorKA.get();
    config.Slot0.kG = climberMotorKG.get();
    config.MotionMagic.MotionMagicExpo_kV = climberMotorKVExpo.get();
    config.MotionMagic.MotionMagicExpo_kA = climberMotorKAExpo.get();
    config.MotionMagic.MotionMagicCruiseVelocity = climberCruiseVelocity.get();

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    SoftwareLimitSwitchConfigs angleMotorLimitSwitches = config.SoftwareLimitSwitch;
    angleMotorLimitSwitches.ReverseSoftLimitEnable = true;
    angleMotorLimitSwitches.ReverseSoftLimitThreshold = MIN_ANGLE_DEGREES.in(Rotations);
    angleMotorLimitSwitches.ForwardSoftLimitEnable = true;
    angleMotorLimitSwitches.ForwardSoftLimitThreshold = MAX_ANGLE_DEGREES.in(Rotations);

    config.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = ClimberConstants.ENCODER_GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;

    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;

    config.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleMotor", motor);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleCANcoder", angleEncoder);
  }
}
