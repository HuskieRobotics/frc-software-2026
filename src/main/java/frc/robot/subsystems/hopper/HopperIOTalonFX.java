package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
  private TalonFX spindexerMotor;
  private TalonFX kickerMotor;

  private VelocitySystemSim spindexerVelocitySystemSim;
  private FlywheelSystemSim kickerSim;

  private VelocityTorqueCurrentFOC spindexerVelocityRequest;
  private VelocityTorqueCurrentFOC kickerVelocityRequest;

  private TorqueCurrentFOC spindexerCurrentRequest;
  private TorqueCurrentFOC kickerCurrentRequest;

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

  private AngularVelocity spindexerReferenceVelocity = RotationsPerSecond.of(0.0);
  private AngularVelocity kickerReferenceVelocity = RotationsPerSecond.of(0.0);

  private Alert spindexerConfigAlert =
      new Alert("Failed to apply configuration for hopper spindexer.", AlertType.kError);

  private Alert kickerConfigAlert =
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
    spindexerMotor = new TalonFX(SPINDEXER_ID, RobotConfig.getInstance().getCANBus());
    kickerMotor = new TalonFX(KICKER_ID, RobotConfig.getInstance().getCANBus());

    spindexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    kickerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    spindexerCurrentRequest = new TorqueCurrentFOC(0.0);
    kickerCurrentRequest = new TorqueCurrentFOC(0.0);

    statorCurrentSpindexerStatusSignal = spindexerMotor.getStatorCurrent();
    supplyCurrentSpindexerStatusSignal = spindexerMotor.getSupplyCurrent();
    tempSpindexerStatusSignal = spindexerMotor.getDeviceTemp();
    voltageSuppliedSpindexerStatusSignal = spindexerMotor.getMotorVoltage();
    velocitySpindexerStatusSignal = spindexerMotor.getVelocity();

    statorCurrentKickerStatusSignal = kickerMotor.getStatorCurrent();
    supplyCurrentKickerStatusSignal = kickerMotor.getSupplyCurrent();
    tempKickerStatusSignal = kickerMotor.getDeviceTemp();
    voltageSuppliedKickerStatusSignal = kickerMotor.getMotorVoltage();
    velocityKickerStatusSignal = kickerMotor.getVelocity();

    Phoenix6Util.registerSignals(
        true,
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

    configSpindexerMotor(spindexerMotor);
    configKickerMotor(kickerMotor);

    spindexerVelocitySystemSim =
        new VelocitySystemSim(SPINDEXER_KV, SPINDEXER_KA, SPINDEXER_GEAR_RATIO, spindexerMotor);

    kickerSim =
        new FlywheelSystemSim(KICKER_KV, KICKER_KA, KICKER_GEAR_RATIO, KICKER_MOI, kickerMotor);
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
    inputs.spindexerReferenceVelocity = this.spindexerReferenceVelocity.copy();

    inputs.kickerStatorCurrent = statorCurrentKickerStatusSignal.getValue();
    inputs.kickerSupplyCurrent = supplyCurrentKickerStatusSignal.getValue();
    inputs.kickerTemperatureCelsius = tempKickerStatusSignal.getValue();
    inputs.kickerVoltageSupplied = voltageSuppliedKickerStatusSignal.getValue();
    inputs.kickerVelocity = velocityKickerStatusSignal.getValue();
    inputs.kickerReferenceVelocity = this.kickerReferenceVelocity.copy();

    if (Constants.TUNING_MODE) {
      inputs.closedLoopErrorSpindexer =
          RotationsPerSecond.of(spindexerMotor.getClosedLoopError().getValueAsDouble());
      inputs.closedLoopErrorKicker =
          RotationsPerSecond.of(kickerMotor.getClosedLoopError().getValueAsDouble());

      inputs.closedLoopReferenceVelocitySpindexer =
          RotationsPerSecond.of(spindexerMotor.getClosedLoopReference().getValueAsDouble());
      inputs.closedLoopReferenceVelocityKicker =
          RotationsPerSecond.of(kickerMotor.getClosedLoopReference().getValueAsDouble());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.kickerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kV = pid[3];
          config.kS = pid[4];
          config.kA = pid[5];

          this.kickerMotor.getConfigurator().apply(config);
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
          this.spindexerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kV = pid[3];
          config.kS = pid[4];
          config.kA = pid[5];

          this.spindexerMotor.getConfigurator().apply(config);
        },
        spindexerMotorKP,
        spindexerMotorKI,
        spindexerMotorKD,
        spindexerMotorKV,
        spindexerMotorKS,
        spindexerMotorKA);

    if (Constants.getMode() == Constants.Mode.SIM) {
      spindexerVelocitySystemSim.updateSim();
      kickerSim.updateSim();
    }
  }

  @Override
  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerMotor.setControl(spindexerVelocityRequest.withVelocity(velocity));
    this.spindexerReferenceVelocity = velocity.copy();
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    kickerMotor.setControl(kickerVelocityRequest.withVelocity(velocity));
    this.kickerReferenceVelocity = velocity.copy();
  }

  @Override
  public void setSpindexerCurrent(Current amps) {
    spindexerMotor.setControl(spindexerCurrentRequest.withOutput(amps));
  }

  @Override
  public void setKickerCurrent(Current amps) {
    kickerMotor.setControl(kickerCurrentRequest.withOutput(amps));
  }

  private void configSpindexerMotor(TalonFX motor) {
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

    Phoenix6Util.applyAndCheckConfiguration(motor, config, spindexerConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper spindexer motor", motor);
  }

  private void configKickerMotor(TalonFX motor) {
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

    Phoenix6Util.applyAndCheckConfiguration(motor, config, kickerConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper kicker motor", motor);
  }
}
