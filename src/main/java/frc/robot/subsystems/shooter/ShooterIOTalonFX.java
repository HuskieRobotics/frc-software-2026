package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import javax.swing.text.Position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

// We usually use VelocityTorqueCurrentFOC to control the velocity of a wheel.
private VelocityTorqueCurrentFOC flywheelLeadVelocityRequest;    ;
//private TorqueCurrentFOC flywheelLeadCurrentRequest;

private VoltageOut kickerVoltageRequest;
private VelocityTorqueCurrentFOC kickerCurrentRequest;
//private VoltageOut kickerUnjamVoltageRequest;

private MotionMagicExpoVoltage turretPositionRequest;
//private VoltageOut turretVoltageRequest;

private MotionMagicExpoVoltage hoodPositionRequest;
private VoltageOut hoodVoltageRequest;

private StatusSignal<Current> flywheelLeadSupplyCurrentStatusSignal;
private StatusSignal<Current> flywheelLeadStatorCurrentStatusSignal;
private StatusSignal<Current> flywheelLeadTorqueCurrentStatusSignal;

private StatusSignal<Current> kickerStatorCurrentStatusSignal;
private StatusSignal<Current> kickerSupplyCurrentStatusSignal;
private StatusSignal<Current> turretStatorCurrentStatusSignal;
private StatusSignal<Current> turretSupplyCurrentStatusSignal;
private StatusSignal<Current> hoodStatorCurrentStatusSignal;
private StatusSignal<Current> hoodSupplyCurrentStatusSignal;

// Angular Velocity Status Signals
// For flywheel lead motor
private StatusSignal<AngularVelocity> flywheelLeadVelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelLeadReferenceVelocityStatusSignal;
//Flywheel closed loop velocity signals as Double due to API limitations
private StatusSignal<Double> flywheelLeadClosedLoopReferenceVelocityStatusSignal;
private StatusSignal<Double> flywheelLeadClosedLoopErrorVelocityStatusSignal;

private StatusSignal<Temperature> flywheelLeadTemperatureStatusSignal;
private StatusSignal<Temperature> kickerTemperatureStatusSignal;
private StatusSignal<Temperature> turretTemperatureStatusSignal;
private StatusSignal<Temperature> hoodTemperatureStatusSignal;

private StatusSignal<Voltage> flywheelLeadVoltageStatusSignal;
private StatusSignal<Voltage> kickerVoltageStatusSignal;
private StatusSignal<Voltage> turretVoltageStatusSignal;
private StatusSignal<Voltage> hoodVoltageStatusSignal;

private StatusSignal<Angle> turretPositionStatusSignal;
private StatusSignal<Angle> hoodPositionStatusSignal;

private AngularVelocity flywheelLeadVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);

private AngularVelocity kickerVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity kickerReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity kickerClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity kickerClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);

private final Debouncer flywheelLeadConnectedDebouncer = new Debouncer(0.5);
private final Debouncer kickerConnectedDebouncer = new Debouncer(0.5);
private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);
private final Debouncer turretConnectedDebouncer = new Debouncer(0.5);

private TalonFX flywheelLead;
private TalonFX kicker;
private TalonFX turret;
private TalonFX hood;

private Alert flywheelLeadConfigAlert =
    new Alert("Failed to apply configuration for fly wheel lead motor.", AlertType.kError);
private Alert flywheelFollow1ConfigAlert =
    new Alert("Failed to apply configuration for fly wheel follow motor.", AlertType.kError);
private Alert flywheelFollow2ConfigAlert =
    new Alert("Failed to apply configuration for fly wheel follow motor.", AlertType.kError);
private Alert flywheelFollow3ConfigAlert =
    new Alert("Failed to apply configuration for fly wheel follow motor.", AlertType.kError);
private Alert kickerConfigAlert =
    new Alert("Failed to apply configuration for kicker motor.", AlertType.kError);
private Alert hoodConfigAlert =
    new Alert("Failed to apply configuration for hood motor.", AlertType.kError);
private Alert turretConfigAlert =
    new Alert("Failed to apply configuration for turret motor.", AlertType.kError);

// The following enables tuning of the PID and feedforward values for the arm by changing values
// via AdvantageScope and not needing to change values in code, compile, and re-deploy.
private final LoggedTunableNumber flywheelLeadKP =
    new LoggedTunableNumber("Shooter/Top kP", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KP);
private final LoggedTunableNumber flywheelLeadKI =
    new LoggedTunableNumber("Shooter/Top kI", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KI);
private final LoggedTunableNumber flywheelLeadKD =
    new LoggedTunableNumber("Shooter/Top kD", ShooterConstants.FLYWHEEL_LEAD_ROTATION_KD);
private final LoggedTunableNumber turretKP =
    new LoggedTunableNumber("Shooter/Turret kP", ShooterConstants.TURRET_ROTATION_KP);
private final LoggedTunableNumber turretKI =
    new LoggedTunableNumber("Shooter/Turret kI", ShooterConstants.TURRET_ROTATION_KI);
private final LoggedTunableNumber turretKD =
    new LoggedTunableNumber("Shooter/Turret kD", ShooterConstants.TURRET_ROTATION_KD);
private final LoggedTunableNumber hoodKP =
    new LoggedTunableNumber("Shooter/Hood kP", ShooterConstants.HOOD_ROTATION_KP);
private final LoggedTunableNumber hoodKI =
    new LoggedTunableNumber("Shooter/Hood kI", ShooterConstants.HOOD_ROTATION_KI);
private final LoggedTunableNumber hoodKD =
    new LoggedTunableNumber("Shooter/Hood kD", ShooterConstants.HOOD_ROTATION_KD);
private final LoggedTunableNumber kickerKP =
    new LoggedTunableNumber("Shooter/Kicker kP", ShooterConstants.KICKER_ROTATION_KP);
private final LoggedTunableNumber kickerKI =
    new LoggedTunableNumber("Shooter/Kicker kI", ShooterConstants.KICKER_ROTATION_KI);
private final LoggedTunableNumber kickerKD =
    new LoggedTunableNumber("Shooter/Kicker kD", ShooterConstants.KICKER_ROTATION_KD);
// It is a bit more challenging to simulate a CANrange sensor compared to a DIO sensor. Using a
// Tunable to simulate the distance to a game piece, requires that TUNING is set to true.
private final LoggedTunableNumber simDetectorDistance =
    new LoggedTunableNumber("Shooter/Sim Detector Distance (m)", 1.0);

private TalonFX shootMotorTop;
private TalonFX shootMotorBottom;
private CANrange gamePieceDetector;

private VelocitySystemSim flywheelLeadSim;
private VelocitySystemSim kickerLeadSim;
private VelocitySystemSim turretLeadSim;
private VelocitySystemSim hoodLeadSim;

public ShooterIOTalonFX() {
  flywheelLead = new TalonFX(FLYWHEEL_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  kicker = new TalonFX(KICKER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  turret = new TalonFX(TURRET_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  hood = new TalonFX(HOOD_MOTOR_ID, RobotConfig.getInstance().getCANBus());

  flywheelLeadVelocityRequest = new VelocityTorqueCurrentFOC(0);
  //flywheelLeadCurrentRequest = new TorqueCurrentFOC(0);

// FLYWHEEL LEAD
flywheelLeadSupplyCurrentStatusSignal = flywheelLead.getSupplyCurrent();
flywheelLeadStatorCurrentStatusSignal = flywheelLead.getStatorCurrent();
flywheelLeadTorqueCurrentStatusSignal = flywheelLead.getTorqueCurrent(); // FIXME: check if it is torque current
flywheelLeadVelocityStatusSignal = flywheelLead.getVelocity();
flywheelLeadReferenceVelocityStatusSignal = flywheelLead.getVelocity(); // reference, cached
flywheelLeadClosedLoopReferenceVelocityStatusSignal = flywheelLead.getClosedLoopReference(); // FIXME: check API
flywheelLeadClosedLoopErrorVelocityStatusSignal = flywheelLead.getClosedLoopError(); // FIXME: check API
flywheelLeadTemperatureStatusSignal = flywheelLead.getDeviceTemp();
flywheelLeadVoltageStatusSignal = flywheelLead.getMotorVoltage();

// KICKER
kickerStatorCurrentStatusSignal = kicker.getStatorCurrent();
kickerSupplyCurrentStatusSignal = kicker.getSupplyCurrent();
kickerTemperatureStatusSignal = kicker.getDeviceTemp();
kickerVoltageStatusSignal = kicker.getMotorVoltage();

// TURRET
turretStatorCurrentStatusSignal = turret.getStatorCurrent();
turretSupplyCurrentStatusSignal = turret.getSupplyCurrent();
turretTemperatureStatusSignal = turret.getDeviceTemp();
turretVoltageStatusSignal = turret.getMotorVoltage();
turretPositionStatusSignal = turret.getPosition();

// HOOD
hoodStatorCurrentStatusSignal = hood.getStatorCurrent();
hoodTemperatureStatusSignal = hood.getDeviceTemp();
hoodVoltageStatusSignal = hood.getMotorVoltage();
hoodPositionStatusSignal = hood.getPosition();


  // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
  // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
  // need to refresh any StatusSignals in this class.
  Phoenix6Util.registerSignals(
      true,
      //FLYWHEEL LEAD
      flywheelLeadSupplyCurrentStatusSignal,
      flywheelLeadStatorCurrentStatusSignal,
      flywheelLeadTorqueCurrentStatusSignal,
      flywheelLeadVelocityStatusSignal,
      flywheelLeadReferenceVelocityStatusSignal,
      flywheelLeadClosedLoopReferenceVelocityStatusSignal,
      flywheelLeadClosedLoopErrorVelocityStatusSignal,
      flywheelLeadTemperatureStatusSignal,
      flywheelLeadVoltageStatusSignal,

      //KICKER
      kickerStatorCurrentStatusSignal,
      kickerSupplyCurrentStatusSignal,
      kickerTemperatureStatusSignal,
      kickerVoltageStatusSignal,

      //TURRET
      turretStatorCurrentStatusSignal,
      turretSupplyCurrentStatusSignal,
      turretTemperatureStatusSignal,
      turretVoltageStatusSignal,
      turretPositionStatusSignal,

      //HOOD
      hoodStatorCurrentStatusSignal,
      hoodSupplyCurrentStatusSignal,
      hoodTemperatureStatusSignal,
      hoodVoltageStatusSignal,
      hoodPositionStatusSignal

      );
  configShootMotor(flywheelLead, FLYWHEEL_LEAD_INVERTED, true, flywheelLeadConfigAlert);
  configShootMotor(kicker, KICKER_INVERTED, false, kickerConfigAlert);
  configShootMotor(turret, TURRET_INVERTED, false, turretConfigAlert);
  configShootMotor(hood, HOOD_INVERTED, false, hoodConfigAlert);
  // Create a simulation objects for the shooter. The specific parameters for the simulation
  // are determined based on the mechanical design of the shooter.
  this.flywheelLeadSim =
      new VelocitySystemSim(
          flywheelLead,
          ShooterConstants.FLYWHEEL_LEAD_INVERTED,
          0.05, //update as needed
          0.01, //up  date as needed
          ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO);
  this.kickerLeadSim =
      new VelocitySystemSim(
          shootMotorTop,
          ShooterConstants.KICKER_INVERTED,
          0.05,
          0.01,
          ShooterConstants.KICKER_GEAR_RATIO);
  this.turretLeadSim =
      new VelocitySystemSim(
          shootMotorBottom,
          ShooterConstants.TURRET_INVERTED,
          0.05,
          0.01,
          ShooterConstants.TURRET_GEAR_RATIO);
  this.hoodLeadSim =
      new VelocitySystemSim(
          shootMotorBottom,
          ShooterConstants.HOOD_INVERTED,
          0.05,
          0.01,
          ShooterConstants.HOOD_GEAR_RATIO);
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
              flywheelLeadVoltageStatusSignal
              ));
  inputs.kickerConnected =
      kickerConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              kickerStatorCurrentStatusSignal,
              kickerSupplyCurrentStatusSignal,
              kickerTemperatureStatusSignal,
              kickerVoltageStatusSignal));
  inputs.hoodConnected =
      hoodConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              hoodStatorCurrentStatusSignal,
              hoodSupplyCurrentStatusSignal,
              hoodTemperatureStatusSignal,
              hoodVoltageStatusSignal
          ));
  inputs.turretConnected =
      turretConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              turretStatorCurrentStatusSignal,
              turretSupplyCurrentStatusSignal,
              turretTemperatureStatusSignal,
              turretVoltageStatusSignal
          ));

  // Updates Flywheel Lead Motor Inputs
  inputs.flywheelLeadStatorCurrent = flywheelLeadStatorCurrentStatusSignal.getValue();
  inputs.flywheelLeadSupplyCurrent = flywheelLeadSupplyCurrentStatusSignal.getValue();
  inputs.flywheelLeadTorqueCurrent = flywheelLeadTorqueCurrentStatusSignal.getValue();
  inputs.flywheelLeadVelocity = flywheelLeadVelocityStatusSignal.getValue();
  inputs.flywheelLeadReferenceVelocity = flywheelLeadReferenceVelocityStatusSignal.getValue();
  inputs.flywheelLeadTemperature = flywheelLeadTemperatureStatusSignal.getValue();
  inputs.flywheelLeadVoltage = flywheelLeadVoltageStatusSignal.getValue();
  inputs.flywheelLeadReferenceVelocity = flywheelLeadReferenceVelocityStatusSignal.getValue();
  inputs.flywheelLeadClosedLoopReferenceVelocity = flywheelLeadClosedLoopReferenceVelocityStatusSignal.getValue();
  inputs.flywheelLeadClosedLoopErrorVelocity = flywheelLeadClosedLoopErrorVelocityStatusSignal.getValue();

  // Updates Kicker Motor Inputs
  inputs.kickerStatorCurrent = kickerStatorCurrentStatusSignal.getValue();
  inputs.kickerSupplyCurrent = kickerSupplyCurrentStatusSignal.getValue();
  inputs.kickerTemperature = kickerTemperatureStatusSignal.getValue();
  inputs.kickerVoltage = kickerVoltageStatusSignal.getValue();

  // Updates Turret Motor Inputs
  inputs.turretStatorCurrent = turretStatorCurrentStatusSignal.getValue();
  inputs.turretSupplyCurrent = turretSupplyCurrentStatusSignal.getValue();
  inputs.turretTemperature = turretTemperatureStatusSignal.getValue();
  inputs.turretVoltage = turretVoltageStatusSignal.getValue();
  inputs.turretPosition = turretPositionStatusSignal.getValue();

  // Updates Hood Motor Inputs
  inputs.hoodStatorCurrent = hoodStatorCurrentStatusSignal.getValue();
  inputs.hoodSupplyCurrent = hoodSupplyCurrentStatusSignal.getValue();
  inputs.hoodTemperature = hoodTemperatureStatusSignal.getValue();
  inputs.hoodVoltage = hoodVoltageStatusSignal.getValue();
  inputs.hoodPosition = hoodPositionStatusSignal.getValue();


  // Retrieve the closed loop reference status signals directly from the motor in this method
  // instead of retrieving in advance because the status signal returned depends on the current
  // control mode. To eliminate the performance hit, only retrieve the closed loop reference
  // signals if the tuning mode is enabled. It is critical that these input values are only used
  // for tuning and not used elsewhere in the subsystem. For example, the
  // shootMotorTopReferenceVelocityRPS property should be used throughout the subsystem since it
  // will always be populated.
  if (Constants.TUNING_MODE) {
    inputs.flywheelLeadVelocity = 
        flywheelLead.getVelocity().getValue();
    inputs.flywheelLeadSupplyCurrent = 
        flywheelLead.getSupplyCurrent().getValue();
    inputs.flywheelLeadStatorCurrent =
        flywheelLead.getStatorCurrent().getValue();
    inputs.kickerVelocity =
        kicker.getVelocity().getValue();
    inputs.kickerSupplyCurrent =
        kicker.getSupplyCurrent().getValue();
    inputs.kickerStatorCurrent = 
        kicker.getStatorCurrent().getValue();
    inputs.turretVoltage =
        turret.getMotorVoltage().getValue();
    inputs.turretSupplyCurrent
    
        // amar
        // left off here need to find getmotorvoltage and change it getvoltage for consistancy


      inputs.flywheelLeadClosedLoopReferenceVelocity =
          flywheelLead.getClosedLoopReference().getValue();
      inputs.flywheelLeadClosedLoopErrorVelocity =
          flywheelLead.getClosedLoopError().getValue();
      inputs.kickerClosedLoopReferenceVelocity =
          RotationsPerSecond.of(kicker.getClosedLoopReference().getValue());
      inputs.kickerClosedLoopErrorVelocity =
          RotationsPerSecond.of(kicker.getClosedLoopError().getValue());
      inputs.turretClosedLoopReferencePosition =
          Rotations.of(turret.getClosedLoopReference().getValue());
      inputs.turretClosedLoopErrorPosition =
          Rotations.of(kicker.getClosedLoopError().getValue());
      inputs.hoodClosedLoopReferencePosition =
          Rotations.of(kicker.getClosedLoopReference().getValue());
      inputs.hoodClosedLoopErrorPosition =
          Rotations.of(kicker.getClosedLoopError().getValue());
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
        flywheelLead.getConfigurator().apply(config);
      },
      flywheelLeadKP,
      flywheelLeadKI,
      flywheelLeadKD);
  LoggedTunableNumber.ifChanged(
      hashCode(),
      pid -> {
        Slot0Configs config = new Slot0Configs();
        kicker.getConfigurator().refresh(config);
        config.kP = pid[0];
        config.kI = pid[1];
        config.kD = pid[2];

        kicker.getConfigurator().apply(config);
      },
      kickerKP,
      kickerKI,
      kickerKD);

  // The last step in the updateInputs method is to update the simulation.
  if (Constants.getMode() == Constants.Mode.SIM) {
    flywheelLeadSim.updateSim();
    kickerLeadSim.updateSim();
  }
}



@Override
public void setFlywheelLeadVelocity(AngularVelocity velocity) {
  flywheelLead.setControl(flywheelLeadVelocityRequest.withVelocity(velocity));

  // To improve performance, we store the reference velocity as an instance variable to avoid
  // having to retrieve the status signal object from the device in the updateInputs method.
  this.flywheelLeadReferenceVelocity = velocity.copy();
}

@Override
public void setKickerVoltage(Voltage voltage) {
    kicker.setControl(kickerVoltageRequest.withOutput(voltage));
}

//@Override
//public void setKickerUnjamVoltage(Voltage voltage) {
    //kicker.setControl(kickerUnjamVoltageRequest.withOutput(voltage));
//}

@Override
public void setTurretPosition(Angle position) {
    turret.setControl(turretPositionRequest.withPosition(position));
}

//@Override
//public void setTurretVoltage(Voltage voltage) {
    //turret.setControl(turretVoltageRequest.withOutput(voltage));
//}

@Override
public void setHoodPosition (Angle position) {
    hood.setControl(hoodPositionRequest.withPosition(position));
}

@Override
public void setHoodVoltage(Voltage voltage) {
    hood.setControl(hoodVoltageRequest.withOutput(voltage));
}

private void configShootMotor(
    TalonFX shootMotor, boolean isInverted, boolean isTopMotor, Alert configAlert) {

  TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();

  if (isTopMotor) {
    shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
  } else {
    shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
  }

  if (isTopMotor) {
    shootMotorsConfig.Slot0.kP = shootMotorTopKP.get();
    shootMotorsConfig.Slot0.kI = shootMotorTopKI.get();
    shootMotorsConfig.Slot0.kD = shootMotorTopKD.get();
    shootMotorsConfig.Slot0.kS = shootMotorTopKS.get();
  } else {
    shootMotorsConfig.Slot0.kP = shootMotorBottomKP.get();
    shootMotorsConfig.Slot0.kI = shootMotorBottomKI.get();
    shootMotorsConfig.Slot0.kD = shootMotorBottomKD.get();
    shootMotorsConfig.Slot0.kS = shootMotorBottomKS.get();
  }

  shootMotorsConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

  shootMotorsConfig.MotorOutput.Inverted =
      isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

  // It is critical that devices are successfully configured. The applyAndCheckConfiguration
  // method will apply the configuration, read back the configuration, and ensure that it is
  // correct. If not, it will reattempt five times and eventually, generate an alert.
  Phoenix6Util.applyAndCheckConfiguration(shootMotor, shootMotorsConfig, configAlert);

  // A subsystem needs to register each device with FaultReporter. FaultReporter will check
  // devices for faults periodically when the robot is disabled and generate alerts if any faults
  // are found.
  FaultReporter.getInstance()
      .registerHardware(SUBSYSTEM_NAME, isTopMotor ? "TopMotor" : "BottomMotor", shootMotor);
}

private void configGamePieceDetector(CANrange detector, Alert configAlert) {
  CANrangeConfiguration config = new CANrangeConfiguration();

  // if CANrange has a signal strength of at least 2000, it is a valid measurement
  config.ProximityParams.MinSignalStrengthForValidMeasurement = detectorMinSignalStrength.get();

  // if CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal
  config.ProximityParams.ProximityThreshold = detectorProximityThreshold.get();

  // make the CANrange update as fast as possible at 100 Hz. This requires short-range mode
  config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

  // It is critical that devices are successfully configured. The applyAndCheckConfiguration
  // method will apply the configuration, read back the configuration, and ensure that it is
  // correct. If not, it will reattempt five times and eventually, generate an alert.
  Phoenix6Util.applyAndCheckConfiguration(detector, config, configAlert);

  // A subsystem needs to register each device with FaultReporter. FaultReporter will check
  // devices for faults periodically when the robot is disabled and generate alerts if any faults
  // are found.
  FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "GamePieceDetector", detector);
}
}
