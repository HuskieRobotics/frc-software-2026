package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.FlywheelSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class HopperIOTalonFX implements HopperIO {
  private TalonFX hopperSpindexerMotor;
  private TalonFX hopperKickerMotor;

  private VelocitySystemSim hopperSpindexerVelocitySystemSim;
  private FlywheelSystemSim hopperKickerSim;

  private VelocityTorqueCurrentFOC hopperSpindexerVelocityRequest;
  private VelocityTorqueCurrentFOC hopperKickerVelocityRequest;

  private StatusSignal<Voltage> voltageSuppliedSpindexerStatusSignal;
  private StatusSignal<Voltage> voltageSuppliedKickerStatusSignal;

  private StatusSignal<Current> supplyCurrentSpindexerStatusSignal;
  private StatusSignal<Current> supplyCurrentKickerStatusSignal;

  private StatusSignal<Current> statorCurrentSpindexerStatusSignal;
  private StatusSignal<Current> statorCurrentKickerStatusSignal;

  private StatusSignal<Temperature> tempSpindexerStatusSignal;
  private StatusSignal<Temperature> tempKickerStatusSignal;

  private StatusSignal<AngularVelocity> velocitySpindexerStatusSignal;
  private StatusSignal<AngularVelocity> velocityKickerStatusSignal;

  private Alert hopperSpindexerConfigAlert =
      new Alert("Failed to apply configuration for hopper spindexer.", AlertType.kError);

  private Alert hopperKickerConfigAlert =
      new Alert("Failed to apply configuration for hopper kicker.", AlertType.kError);

  private final LoggedTunableNumber spindexerMotorKP =
      new LoggedTunableNumber("Hopper/SPINDEXER_KP", HopperConstants.SPINDEXER_KP);
  private final LoggedTunableNumber spindexerMotorKI =
      new LoggedTunableNumber("Hopper/SPINDEXER_KI", HopperConstants.SPINDEXER_KI);
  private final LoggedTunableNumber spindexerMotorKD =
      new LoggedTunableNumber("Hopper/SPINDEXER_KD", HopperConstants.SPINDEXER_KD);
  private final LoggedTunableNumber spindexerMotorKV =
      new LoggedTunableNumber("Hopper/SPINDEXER_KV", HopperConstants.SPINDEXER_KV);
  private final LoggedTunableNumber spindexerMotorKS =
      new LoggedTunableNumber("Hopper/SPINDEXER_KS", HopperConstants.SPINDEXER_KS);
  private final LoggedTunableNumber spindexerMotorKA =
      new LoggedTunableNumber("Hopper/SPINDEXER_KA", HopperConstants.SPINDEXER_KA);

  private final LoggedTunableNumber kickerMotorKP =
      new LoggedTunableNumber("Hopper/KICKER_KP", HopperConstants.KICKER_KP);
  private final LoggedTunableNumber kickerMotorKI =
      new LoggedTunableNumber("Hopper/KICKER_KI", HopperConstants.KICKER_KI);
  private final LoggedTunableNumber kickerMotorKD =
      new LoggedTunableNumber("Hopper/KICKER_KD", HopperConstants.KICKER_KD);
  private final LoggedTunableNumber kickerMotorKV =
      new LoggedTunableNumber("Hopper/KICKER_KV", HopperConstants.KICKER_KV);
  private final LoggedTunableNumber kickerMotorKS =
      new LoggedTunableNumber("Hopper/KICKER_KS", HopperConstants.KICKER_KS);
  private final LoggedTunableNumber kickerMotorKA =
      new LoggedTunableNumber("Hopper/KICKER_KA", HopperConstants.KICKER_KA);

  private final Debouncer connectedSpindexerDebouncer = new Debouncer(0.5);
  private final Debouncer connectedKickerDebouncer = new Debouncer(0.5);

  public HopperIOTalonFX() {
    hopperSpindexerMotor = new TalonFX(HOPPER_SPINDEXER_ID, RobotConfig.getInstance().getCANBus());
    hopperKickerMotor = new TalonFX(HOPPER_KICKER_ID, RobotConfig.getInstance().getCANBus());

    statorCurrentSpindexerStatusSignal = hopperSpindexerMotor.getStatorCurrent();
    supplyCurrentSpindexerStatusSignal = hopperSpindexerMotor.getSupplyCurrent();
    tempSpindexerStatusSignal = hopperSpindexerMotor.getDeviceTemp();
    voltageSuppliedSpindexerStatusSignal = hopperSpindexerMotor.getMotorVoltage();
    velocitySpindexerStatusSignal = hopperSpindexerMotor.getVelocity();

    statorCurrentKickerStatusSignal = hopperKickerMotor.getStatorCurrent();
    supplyCurrentKickerStatusSignal = hopperKickerMotor.getSupplyCurrent();
    tempKickerStatusSignal = hopperKickerMotor.getDeviceTemp();
    voltageSuppliedKickerStatusSignal = hopperKickerMotor.getMotorVoltage();
    velocityKickerStatusSignal = hopperKickerMotor.getVelocity();

    hopperSpindexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    hopperKickerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    Phoenix6Util.registerSignals(
        false,
        statorCurrentSpindexerStatusSignal,
        supplyCurrentSpindexerStatusSignal,
        tempSpindexerStatusSignal,
        voltageSuppliedSpindexerStatusSignal,
        velocitySpindexerStatusSignal,
        statorCurrentKickerStatusSignal,
        supplyCurrentKickerStatusSignal,
        tempKickerStatusSignal,
        voltageSuppliedKickerStatusSignal,
        velocityKickerStatusSignal);

    configHopperSpindexerMotor(hopperSpindexerMotor);
    configHopperKickerMotor(hopperKickerMotor);

    hopperSpindexerVelocitySystemSim =
        new VelocitySystemSim(
            SPINDEXER_KV, SPINDEXER_KA, SPINDEXER_GEAR_RATIO, hopperSpindexerMotor);

    hopperKickerSim =
        new FlywheelSystemSim(
            KICKER_KV, KICKER_KA, KICKER_MOI, KICKER_GEAR_RATIO, hopperKickerMotor);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.spindexerMotorConnected =
        connectedSpindexerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                supplyCurrentSpindexerStatusSignal,
                statorCurrentSpindexerStatusSignal,
                tempSpindexerStatusSignal,
                voltageSuppliedSpindexerStatusSignal,
                velocitySpindexerStatusSignal));

    inputs.kickerMotorConnected =
        connectedKickerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                supplyCurrentKickerStatusSignal,
                statorCurrentKickerStatusSignal,
                tempKickerStatusSignal,
                voltageSuppliedKickerStatusSignal,
                velocityKickerStatusSignal));

    inputs.spindexerStatorCurrent = statorCurrentSpindexerStatusSignal.getValue();
    inputs.spindexerSupplyCurrent = supplyCurrentSpindexerStatusSignal.getValue();
    inputs.spindexerTemperatureCelsius = tempSpindexerStatusSignal.getValue();
    inputs.spindexerVoltageSupplied = voltageSuppliedSpindexerStatusSignal.getValue();
    inputs.spindexerVelocity = velocitySpindexerStatusSignal.getValue();

    inputs.kickerStatorCurrent = statorCurrentKickerStatusSignal.getValue();
    inputs.kickerSupplyCurrent = supplyCurrentKickerStatusSignal.getValue();
    inputs.kickerTemperatureCelsius = tempKickerStatusSignal.getValue();
    inputs.kickerVoltageSupplied = voltageSuppliedKickerStatusSignal.getValue();
    inputs.kickerVelocity = velocityKickerStatusSignal.getValue();

    if (Constants.TUNING_MODE) {
      inputs.closedLoopErrorSpindexer =
          RotationsPerSecond.of(hopperSpindexerMotor.getClosedLoopError().getValueAsDouble());
      inputs.closedLoopErrorKicker =
          RotationsPerSecond.of(hopperKickerMotor.getClosedLoopError().getValueAsDouble());

      inputs.closedLoopReferenceSpindexer =
          RotationsPerSecond.of(hopperSpindexerMotor.getClosedLoopReference().getValueAsDouble());
      inputs.closedLoopReferenceKicker =
          RotationsPerSecond.of(hopperKickerMotor.getClosedLoopReference().getValueAsDouble());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.hopperKickerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kV = pid[3];
          config.kS = pid[4];
          config.kA = pid[5];

          this.hopperKickerMotor.getConfigurator().apply(config);
        },
        kickerMotorKP,
        kickerMotorKI,
        kickerMotorKD,
        kickerMotorKV,
        kickerMotorKS,
        kickerMotorKA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.hopperSpindexerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kV = pid[3];
          config.kS = pid[4];
          config.kA = pid[5];

          this.hopperSpindexerMotor.getConfigurator().apply(config);
        },
        spindexerMotorKP,
        spindexerMotorKI,
        spindexerMotorKD,
        spindexerMotorKV,
        spindexerMotorKS,
        spindexerMotorKA);

    hopperSpindexerVelocitySystemSim.updateSim();
    hopperKickerSim.updateSim();
  }

  @Override
  public void setSpindexerVelocity(AngularVelocity velocity) {
    hopperSpindexerMotor.setControl(hopperSpindexerVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    hopperKickerMotor.setControl(hopperKickerVelocityRequest.withVelocity(velocity));
  }

  private void configHopperSpindexerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.TorqueCurrent.PeakForwardTorqueCurrent =
        HopperConstants.SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -HopperConstants.SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT;

    config.Feedback.SensorToMechanismRatio = SPINDEXER_GEAR_RATIO;

    config.MotorOutput.Inverted =
        SPINDEXER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = spindexerMotorKP.get();
    config.Slot0.kI = spindexerMotorKI.get();
    config.Slot0.kD = spindexerMotorKD.get();
    config.Slot0.kV = spindexerMotorKV.get();
    config.Slot0.kS = spindexerMotorKS.get();
    config.Slot0.kA = spindexerMotorKA.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperSpindexerConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper spindexer motor", motor);
  }

  private void configHopperKickerMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.TorqueCurrent.PeakForwardTorqueCurrent = HopperConstants.KICKER_MOTOR_PEAK_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        -HopperConstants.KICKER_MOTOR_PEAK_CURRENT_LIMIT;

    config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

    config.MotorOutput.Inverted =
        KICKER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = kickerMotorKP.get();
    config.Slot0.kI = kickerMotorKI.get();
    config.Slot0.kD = kickerMotorKD.get();
    config.Slot0.kV = kickerMotorKV.get();
    config.Slot0.kS = kickerMotorKS.get();
    config.Slot0.kA = kickerMotorKA.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperKickerConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper kicker motor", motor);
  }
}
