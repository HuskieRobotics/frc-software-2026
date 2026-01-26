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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

// We usually use VelocityTorqueCurrentFOC to control the velocity of a wheel.
private VelocityTorqueCurrentFOC flywheelLeadVelocityRequest;
//private TorqueCurrentFOC flywheelLeadCurrentRequest;

private VoltageOut kickerVoltageRequest;
private VelocityTorqueCurrentFOC kickerCurrentRequest;
//private VoltageOut kickerUnjamVoltageRequest;

private MotionMagicExpoVoltage turretPositionRequest;
//private VoltageOut turretVoltageRequest;

private MotionMagicExpoVoltage hoodPositionRequest;
private VoltageOut hoodVoltageRequest;

//Flywheel Lead Status Signals
private StatusSignal<Current> flywheelLeadSupplyCurrentStatusSignal;
private StatusSignal<Current> flywheelLeadStatorCurrentStatusSignal;
private StatusSignal<Current> flywheelLeadTorqueCurrentStatusSignal;

//Flywheel Follow1 Status Signals
private StatusSignal<Current> flywheelFollow1SupplyCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow1StatorCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow1TorqueCurrentStatusSignal;

//Flywheel Follow2 Status Signals
private StatusSignal<Current> flywheelFollow2SupplyCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow2StatorCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow2TorqueCurrentStatusSignal;

//Flywheel Follow3 Status Signals
private StatusSignal<Current> flywheelFollow3SupplyCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow3StatorCurrentStatusSignal;
private StatusSignal<Current> flywheelFollow3TorqueCurrentStatusSignal;

// Kicker, Turret, and Hood Status Signals
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
private StatusSignal<AngularVelocity> flywheelFollow1VelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelFollow1ReferenceVelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelFollow2VelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelFollow2ReferenceVelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelFollow3VelocityStatusSignal;
private StatusSignal<AngularVelocity> flywheelFollow3ReferenceVelocityStatusSignal;
//Flywheel closed loop velocity signals as Double due to API limitations
private StatusSignal<Double> flywheelLeadClosedLoopReferenceVelocityStatusSignal;
private StatusSignal<Double> flywheelLeadClosedLoopErrorVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow1ClosedLoopReferenceVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow1ClosedLoopErrorVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow2ClosedLoopReferenceVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow2ClosedLoopErrorVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow3ClosedLoopReferenceVelocityStatusSignal;
private StatusSignal<Double> flywheelFollow3ClosedLoopErrorVelocityStatusSignal;

private StatusSignal<Temperature> flywheelLeadTemperatureStatusSignal;
private StatusSignal<Temperature> flywheelFollow1TemperatureStatusSignal;
private StatusSignal<Temperature> flywheelFollow2TemperatureStatusSignal;
private StatusSignal<Temperature> flywheelFollow3TemperatureStatusSignal;
private StatusSignal<Temperature> kickerTemperatureStatusSignal;
private StatusSignal<Temperature> turretTemperatureStatusSignal;
private StatusSignal<Temperature> hoodTemperatureStatusSignal;

private StatusSignal<Voltage> flywheelLeadVoltageStatusSignal;
private StatusSignal<Voltage> flywheelFollow1VoltageStatusSignal;
private StatusSignal<Voltage> flywheelFollow2VoltageStatusSignal;
private StatusSignal<Voltage> flywheelFollow3VoltageStatusSignal;
private StatusSignal<Voltage> kickerVoltageStatusSignal;
private StatusSignal<Voltage> turretVoltageStatusSignal;
private StatusSignal<Voltage> hoodVoltageStatusSignal;

private StatusSignal<Angle> turretPositionStatusSignal;
private StatusSignal<Angle> hoodPositionStatusSignal;

private AngularVelocity flywheelLeadVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
private AngularVelocity flywheelLeadClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);

private Voltage kickerVoltage = Volts.of(0.0);

private final Debouncer flywheelLeadConnectedDebouncer = new Debouncer(0.5);
private final Debouncer kickerConnectedDebouncer = new Debouncer(0.5);
private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);
private final Debouncer turretConnectedDebouncer = new Debouncer(0.5);

private TalonFX flywheelLead;
private TalonFX flywheelFollow1;
private TalonFX flywheelFollow2;
private TalonFX flywheelFollow3;
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
private final LoggedTunableNumber turretKV =
    new LoggedTunableNumber("Shooter/Turret kV", ShooterConstants.TURRET_ROTATION_EXPO_KV);
private final LoggedTunableNumber turretKA =
    new LoggedTunableNumber("Shooter/Turret kA", ShooterConstants.TURRET_ROTATION_EXPO_KA);
private final LoggedTunableNumber turretMotionMagicCruiseVelocity =
    new LoggedTunableNumber("Shooter/Turret Magic Cruise Velocity", ShooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY);
private final LoggedTunableNumber hoodKP =
    new LoggedTunableNumber("Shooter/Hood kP", ShooterConstants.HOOD_ROTATION_KP);
private final LoggedTunableNumber hoodKI =
    new LoggedTunableNumber("Shooter/Hood kI", ShooterConstants.HOOD_ROTATION_KI);
private final LoggedTunableNumber hoodKD =
    new LoggedTunableNumber("Shooter/Hood kD", ShooterConstants.HOOD_ROTATION_KD);
private final LoggedTunableNumber hoodKV =
    new LoggedTunableNumber("Shooter/Hood kV", ShooterConstants.HOOD_ROTATION_EXPO_KV);
private final LoggedTunableNumber hoodKA =
    new LoggedTunableNumber("Shooter/Hood kA", ShooterConstants.HOOD_ROTATION_EXPO_KA);
private final LoggedTunableNumber hoodMotionMagicCruiseVelocity =
    new LoggedTunableNumber("Shooter/Hood Magic Cruise Velocity", ShooterConstants.HOOD_MOTION_MAGIC_CRUISE_VELOCITY);
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

private VelocitySystemSim flywheelLeadSim;
private VelocitySystemSim flywheelFollow1Sim;
private VelocitySystemSim flywheelFollow2Sim;
private VelocitySystemSim flywheelFollow3Sim;
private VelocitySystemSim kickerLeadSim;
private VelocitySystemSim turretLeadSim;
private VelocitySystemSim hoodLeadSim;

public ShooterIOTalonFX() {
  flywheelLead = new TalonFX(FLYWHEEL_LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  flywheelFollow1 = new TalonFX(FLYWHEEL_FOLLOW_1_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  flywheelFollow2 = new TalonFX(FLYWHEEL_FOLLOW_2_MOTOR_ID, RobotConfig.getInstance().getCANBus());
  flywheelFollow3 = new TalonFX(FLYWHEEL_FOLLOW_3_MOTOR_ID, RobotConfig.getInstance().getCANBus());
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

// Follow 1
flywheelFollow1SupplyCurrentStatusSignal = flywheelFollow1.getSupplyCurrent();
flywheelFollow1StatorCurrentStatusSignal = flywheelFollow1.getStatorCurrent();
flywheelFollow1TorqueCurrentStatusSignal = flywheelFollow1.getTorqueCurrent(); // FIXME: check if it is torque current
flywheelFollow1VelocityStatusSignal = flywheelFollow1.getVelocity();
flywheelFollow1ReferenceVelocityStatusSignal = flywheelFollow1.getVelocity(); // reference, cached
flywheelFollow1ClosedLoopReferenceVelocityStatusSignal = flywheelFollow1.getClosedLoopReference(); // FIXME: check API
flywheelFollow1ClosedLoopErrorVelocityStatusSignal = flywheelFollow1.getClosedLoopError(); // FIXME: check API
flywheelFollow1TemperatureStatusSignal = flywheelFollow1.getDeviceTemp();
flywheelFollow1VoltageStatusSignal = flywheelFollow1.getMotorVoltage();


// Follow 2
flywheelFollow2SupplyCurrentStatusSignal = flywheelFollow2.getSupplyCurrent();
flywheelFollow2StatorCurrentStatusSignal = flywheelFollow2.getStatorCurrent();
flywheelFollow2TorqueCurrentStatusSignal = flywheelFollow2.getTorqueCurrent(); // FIXME: check if it is torque current
flywheelFollow2VelocityStatusSignal = flywheelFollow2.getVelocity();
flywheelFollow2ReferenceVelocityStatusSignal = flywheelFollow2.getVelocity(); // reference, cached
flywheelFollow2ClosedLoopReferenceVelocityStatusSignal = flywheelFollow2.getClosedLoopReference(); // FIXME: check API
flywheelFollow2ClosedLoopErrorVelocityStatusSignal = flywheelFollow2.getClosedLoopError(); // FIXME: check API
flywheelFollow2TemperatureStatusSignal = flywheelFollow2.getDeviceTemp();
flywheelFollow2VoltageStatusSignal = flywheelFollow2.getMotorVoltage();


// Follow 3
flywheelFollow3SupplyCurrentStatusSignal = flywheelFollow3.getSupplyCurrent();
flywheelFollow3StatorCurrentStatusSignal = flywheelFollow3.getStatorCurrent();
flywheelFollow3TorqueCurrentStatusSignal = flywheelFollow3.getTorqueCurrent(); // FIXME: check if it is torque current
flywheelFollow3VelocityStatusSignal = flywheelFollow3.getVelocity();
flywheelFollow3ReferenceVelocityStatusSignal = flywheelFollow3.getVelocity(); // reference, cached
flywheelFollow3ClosedLoopReferenceVelocityStatusSignal = flywheelFollow3.getClosedLoopReference(); // FIXME: check API
flywheelFollow3ClosedLoopErrorVelocityStatusSignal = flywheelFollow3.getClosedLoopError(); // FIXME: check API
flywheelFollow3TemperatureStatusSignal = flywheelFollow3.getDeviceTemp();
flywheelFollow3VoltageStatusSignal = flywheelFollow3.getMotorVoltage();


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

      //FLYWHEEL Follow 1
      flywheelFollow1SupplyCurrentStatusSignal,
      flywheelFollow1StatorCurrentStatusSignal,
      flywheelFollow1TorqueCurrentStatusSignal,
      flywheelFollow1VelocityStatusSignal,
      flywheelFollow1ReferenceVelocityStatusSignal,
      flywheelFollow1ClosedLoopReferenceVelocityStatusSignal,
      flywheelFollow1ClosedLoopErrorVelocityStatusSignal,

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
  configFlywheelLead(flywheelLead, FLYWHEEL_LEAD_INVERTED,"flywheel lead", true, flywheelLeadConfigAlert);
  configFlywheelFollow1(flywheelFollow1, FLYWHEEL_FOLLOW_1_INVERTED, "flywheel follow 1", false, flywheelFollow1ConfigAlert);
  configFlywheelFollow2(flywheelFollow2, FLYWHEEL_FOLLOW_2_INVERTED, "flywheel follow 2", false, flywheelFollow2ConfigAlert);
  configFlywheelFollow3(flywheelFollow3, FLYWHEEL_FOLLOW_3_INVERTED, "flywheel follow 3", false, flywheelFollow3ConfigAlert);
  configKicker(kicker, KICKER_INVERTED, "kicker", kickerConfigAlert);
  configTurret(turret, TURRET_INVERTED, "turret", turretConfigAlert);
  configHood(hood, HOOD_INVERTED, "hood", hoodConfigAlert);
  
  // Create a simulation objects for the shooter. The specific parameters for the simulation
  // are determined based on the mechanical design of the shooter.
  this.flywheelLeadSim =
      new VelocitySystemSim(
          flywheelLead,
          ShooterConstants.FLYWHEEL_LEAD_INVERTED,
          0.05, //update as needed
          0.01, //update as needed
          ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO);
 this.flywheelFollow1Sim =
      new VelocitySystemSim(
          flywheelFollow1,
          ShooterConstants.FLYWHEEL_FOLLOW_1_INVERTED,
          0.05, //update as needed
          0.01, //update as needed
          ShooterConstants.FLYWHEEL_FOLLOW_1_GEAR_RATIO);
  this.kickerLeadSim =
      new VelocitySystemSim(
          kicker,
          ShooterConstants.KICKER_INVERTED,
          0.05,
          0.01,
          ShooterConstants.KICKER_GEAR_RATIO);
  this.turretLeadSim =
      new VelocitySystemSim(
          turret,
          ShooterConstants.TURRET_INVERTED,
          0.05,
          0.01,
          ShooterConstants.TURRET_GEAR_RATIO);
  this.hoodLeadSim =
      new VelocitySystemSim(
          hood,
          ShooterConstants.HOOD_INVERTED,
          0.05,
          0.01,
          ShooterConstants.HOOD_GEAR_RATIO);
    this.flywheelFollow1Sim =
      new VelocitySystemSim(
          flywheelFollow1,
          ShooterConstants.FLYWHEEL_FOLLOW_1_INVERTED,
          0.05, //update as needed
          0.01, //update as needed
          ShooterConstants.FLYWHEEL_FOLLOW_1_GEAR_RATIO);
    this.flywheelFollow2Sim =
      new VelocitySystemSim(
          flywheelFollow2,
          ShooterConstants.FLYWHEEL_FOLLOW_2_INVERTED,
          0.05, //update as needed
          0.01, //update as needed
          ShooterConstants.FLYWHEEL_FOLLOW_2_GEAR_RATIO);
    this.flywheelFollow3Sim =
      new VelocitySystemSim(
          flywheelFollow3,
          ShooterConstants.FLYWHEEL_FOLLOW_3_INVERTED,
          0.05, //update as needed
          0.01, //update as needed
          ShooterConstants.FLYWHEEL_FOLLOW_3_GEAR_RATIO);
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
  inputs.flywheelFollower1Connected =
      flywheelLeadConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              flywheelFollow1StatorCurrentStatusSignal,
              flywheelFollow1SupplyCurrentStatusSignal,
              flywheelFollow1TemperatureStatusSignal,
              flywheelFollow1VoltageStatusSignal
              ));
  inputs.flywheelFollower2Connected =
      flywheelLeadConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              flywheelFollow2StatorCurrentStatusSignal,
              flywheelFollow2SupplyCurrentStatusSignal,
              flywheelFollow2TemperatureStatusSignal,
              flywheelFollow2VoltageStatusSignal
              ));
  inputs.flywheelFollower3Connected =
      flywheelLeadConnectedDebouncer.calculate(
          BaseStatusSignal.isAllGood(
              flywheelFollow3StatorCurrentStatusSignal,
              flywheelFollow3SupplyCurrentStatusSignal,
              flywheelFollow3TemperatureStatusSignal,
              flywheelFollow3VoltageStatusSignal
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

  // Updates Flywheel Follow1 Motor Inputs
  inputs.flywheelFollow1StatorCurrent = flywheelFollow1StatorCurrentStatusSignal.getValue();
  inputs.flywheelFollow1SupplyCurrent = flywheelFollow1SupplyCurrentStatusSignal.getValue();
  inputs.flywheelFollow1TorqueCurrent = flywheelFollow1TorqueCurrentStatusSignal.getValue();
  inputs.flywheelFollow1Velocity = flywheelFollow1VelocityStatusSignal.getValue();
  inputs.flywheelFollow1Temperature = flywheelFollow1TemperatureStatusSignal.getValue();
  inputs.flywheelFollow1Voltage = flywheelFollow1VoltageStatusSignal.getValue();
  inputs.flywheelFollow1ReferenceVelocity = flywheelFollow1ReferenceVelocityStatusSignal.getValue();

// Updates Flywheel Follow2 Motor Inputs
inputs.flywheelFollow2StatorCurrent = flywheelFollow2StatorCurrentStatusSignal.getValue();
inputs.flywheelFollow2SupplyCurrent = flywheelFollow2SupplyCurrentStatusSignal.getValue();
inputs.flywheelFollow2TorqueCurrent = flywheelFollow2TorqueCurrentStatusSignal.getValue();
inputs.flywheelFollow2Velocity = flywheelFollow2VelocityStatusSignal.getValue();
inputs.flywheelFollow2Temperature = flywheelFollow2TemperatureStatusSignal.getValue();
inputs.flywheelFollow2Voltage = flywheelFollow2VoltageStatusSignal.getValue();
inputs.flywheelFollow2ReferenceVelocity = flywheelFollow2ReferenceVelocityStatusSignal.getValue();

// Updates Flywheel Follow3 Motor Inputs
inputs.flywheelFollow3StatorCurrent = flywheelFollow3StatorCurrentStatusSignal.getValue();
inputs.flywheelFollow3SupplyCurrent = flywheelFollow3SupplyCurrentStatusSignal.getValue();
inputs.flywheelFollow3TorqueCurrent = flywheelFollow3TorqueCurrentStatusSignal.getValue();
inputs.flywheelFollow3Velocity = flywheelFollow3VelocityStatusSignal.getValue();
inputs.flywheelFollow3Temperature = flywheelFollow3TemperatureStatusSignal.getValue();
inputs.flywheelFollow3Voltage = flywheelFollow3VoltageStatusSignal.getValue();
inputs.flywheelFollow3ReferenceVelocity = flywheelFollow3ReferenceVelocityStatusSignal.getValue();

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
    //Flywheel Lead
    inputs.flywheelLeadVelocity = 
        flywheelLead.getVelocity().getValue();
    inputs.flywheelLeadSupplyCurrent = 
        flywheelLead.getSupplyCurrent().getValue();
    inputs.flywheelLeadStatorCurrent =
        flywheelLead.getStatorCurrent().getValue();
    inputs.flywheelLeadClosedLoopReferenceVelocity =
        RotationsPerSecond.of(flywheelLead.getClosedLoopReference().getValue());
    inputs.flywheelLeadClosedLoopErrorVelocity =
        RotationsPerSecond.of(flywheelLead.getClosedLoopError().getValue());
    
    //Kicker
    inputs.kickerVoltage =
        kicker.getMotorVoltage().getValue();
    inputs.kickerSupplyCurrent =
        kicker.getSupplyCurrent().getValue();
    inputs.kickerStatorCurrent = 
        kicker.getStatorCurrent().getValue();

    //Turret
    inputs.turretVoltage =
        turret.getMotorVoltage().getValue();
    inputs.turretSupplyCurrent =
        turret.getSupplyCurrent().getValue();
    inputs.turretStatorCurrent =
        turret.getStatorCurrent().getValue();
    //Hood
    inputs.hoodVoltage =
        hood.getMotorVoltage().getValue();
    inputs.hoodSupplyCurrent =
        hood.getSupplyCurrent().getValue();
    inputs.hoodStatorCurrent =
        hood.getStatorCurrent().getValue();
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
  LoggedTunableNumber.ifChanged(
      hashCode(),
      pid -> {
        Slot0Configs config = new Slot0Configs();
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        
        turret.getConfigurator().refresh(config);
        turret.getConfigurator().refresh(mmConfig);
        
        config.kP = pid[0];
        config.kI = pid[1];
        config.kD = pid[2];
        config.kV = pid[3];
        config.kA = pid[4]; 
        mmConfig.MotionMagicCruiseVelocity = pid[5];

        turret.getConfigurator().apply(config);
        turret.getConfigurator().apply(mmConfig);
        
      },
      turretKP,
      turretKI,
      turretKD,
      turretKV,
      turretKA,
      turretMotionMagicCruiseVelocity);

  LoggedTunableNumber.ifChanged(
    hashCode(),
    pid -> {
      Slot0Configs config = new Slot0Configs();
      MotionMagicConfigs mmConfig = new MotionMagicConfigs();
      hood.getConfigurator().refresh(config);
      config.kP = pid[0];
      config.kI = pid[1];
      config.kD = pid[2];
      config.kV = pid[3];
      config.kA = pid[4];
      mmConfig.MotionMagicCruiseVelocity = pid[5];

      hood.getConfigurator().apply(config);
      hood.getConfigurator().apply(mmConfig);

    },
    hoodKP,
    hoodKI,
    hoodKD,
    turretKV,
    turretKA,
    turretMotionMagicCruiseVelocity);

  // The last step in the updateInputs method is to update the simulation.
  if (Constants.getMode() == Constants.Mode.SIM) {
    flywheelLeadSim.updateSim();
    kickerLeadSim.updateSim();
    turretLeadSim.updateSim();
    hoodLeadSim.updateSim();
    flywheelFollow1Sim.updateSim();
    flywheelFollow2Sim.updateSim();
    flywheelFollow3Sim.updateSim();
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

private void configFlywheelLead(
    TalonFX flywheel, boolean isInverted, boolean isLead, Alert configAlert) {

  TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.FLYWHEEL_LEAD_PEAK_CURRENT_LIMIT;
    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.FLYWHEEL_LEAD_PEAK_CURRENT_LIMIT;
    
    flywheelConfig.Slot0.kP = flywheelLeadKP.get();
    flywheelConfig.Slot0.kI = flywheelLeadKI.get();
    flywheelConfig.Slot0.kD = flywheelLeadKD.get();

  flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_LEAD_GEAR_RATIO; //FIXME: would we need this as ratio is 1:1

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
  FaultReporter.getInstance()
      .registerHardware(SUBSYSTEM_NAME, isLead ? "TopMotor" : "BottomMotor", flywheel);
}

private void configKicker(
    TalonFX kicker, boolean isInverted, boolean isLead, Alert configAlert) {

  TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    kickerConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.KICKER_PEAK_CURRENT_LIMIT;
    kickerConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.KICKER_PEAK_CURRENT_LIMIT;
    
    kickerConfig.Slot0.kP = kickerKP.get();
    kickerConfig.Slot0.kI = kickerKI.get();
    kickerConfig.Slot0.kD = kickerKD.get();

  kickerConfig.Feedback.SensorToMechanismRatio = ShooterConstants.KICKER_GEAR_RATIO; //FIXME: would we need this?

  kickerConfig.MotorOutput.Inverted =
      isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
}

private void configTurret(
    TalonFX turret, boolean isInverted, boolean isLead, Alert configAlert) {

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();

        turretConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            ShooterConstants.TURRET_PEAK_CURRENT_LIMIT;
        turretConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            -ShooterConstants.TURRET_PEAK_CURRENT_LIMIT;
        
        turretConfig.Slot0.kP = turretKP.get();
        turretConfig.Slot0.kI = turretKI.get();
        turretConfig.Slot0.kD = turretKD.get();
        turretConfig.Slot0.kV = turretKV.get();
        turretConfig.Slot0.kA = turretKA.get();
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretMotionMagicCruiseVelocity.get();
    
    turretConfig.Feedback.SensorToMechanismRatio = ShooterConstants.TURRET_GEAR_RATIO; //FIXME: would we need this?

    turretConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
}

private void configHood(
    TalonFX hood, boolean isInverted, boolean isLead, Alert configAlert) {

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); 
        
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            ShooterConstants.HOOD_PEAK_CURRENT_LIMIT;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            -ShooterConstants.HOOD_PEAK_CURRENT_LIMIT;

        hoodConfig.Slot0.kP = hoodKP.get();
        hoodConfig.Slot0.kI = hoodKI.get();
        hoodConfig.Slot0.kD = hoodKD.get();
        hoodConfig.Slot0.kV = hoodKV.get();
        hoodConfig.Slot0.kA = hoodKA.get();
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = hoodMotionMagicCruiseVelocity.get();

    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEAR_RATIO; //FIXME: would we need this?

    hoodConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
}
    
    


  // It is critical that devices are successfully configured. The applyAndCheckConfiguration
  // method will apply the configuration, read back the configuration, and ensure that it is
  // correct. If not, it will reattempt five times and eventually, generate an alert.
  Phoenix6Util.applyAndCheckConfiguration(flywheel, flywheelConfig, configAlert);
  Phoenix6Util.applyAndCheckConfiguration(kicker, kickerConfig, configAlert);
  Phoenix6Util.applyAndCheckConfiguration(hood, hoodConfig, configAlert);
  Phoenix6Util.applyAndCheckConfiguration(turret, turretConfig, configAlert);

  // A subsystem needs to register each device with FaultReporter. FaultReporter will check
  // devices for faults periodically when the robot is disabled and generate alerts if any faults
  // are found.
  FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, kicker); //FIXME: add names and adjust to other motors 
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
} // FIXME: fix errors for brackets

