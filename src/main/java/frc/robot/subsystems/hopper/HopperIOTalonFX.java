package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.elevator.ElevatorConstants.SUBSYSTEM_NAME;
import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;

public class HopperIOTalonFX implements HopperIO {
    private TalonFX hopperSpindexerMotor;
    private TalonFX hopperRollerMotor;

    private VoltageOut hopperSpindexerVoltageRequest;
    private VoltageOut hopperRollerVoltageRequest;
    
    private StatusSignal<Voltage> voltageSuppliedSpindexer;
    private StatusSignal<Voltage> voltageSuppliedRoller;

    private StatusSignal<Current> supplyCurrentSpindexer;
    private StatusSignal<Current> supplyCurrentRoller;

    private StatusSignal<Current> statorCurrentSpindexer;
    private StatusSignal<Current> statorCurrentRoller;

    private StatusSignal<Temperature> tempSpindexer;
    private StatusSignal<Temperature> tempRoller;

    private Alert hopperSpindexerConfigAlert =
      new Alert("Failed to apply configuration for hopper spindexer.", AlertType.kError);

    private Alert hopperRollerConfigAlert =
      new Alert("Failed to apply configuration for hopper roller.", AlertType.kError);

    private final Debouncer connectedSpindexerDebouncer = new Debouncer(0.5);
    private final Debouncer connectedRollerDebouncer = new Debouncer(0.5);

    public HopperIOTalonFX() {
        hopperSpindexerMotor = new TalonFX(HOPPER_SPINDEXER_ID);
        hopperRollerMotor = new TalonFX(HOPPER_ROLLER_ID);

        hopperSpindexerVoltageRequest = new VoltageOut(0.0);
        hopperRollerVoltageRequest = new VoltageOut(0.0);

        statorCurrentSpindexer = hopperSpindexerMotor.getStatorCurrent();
        supplyCurrentSpindexer = hopperSpindexerMotor.getSupplyCurrent();
        tempSpindexer = hopperSpindexerMotor.getDeviceTemp();
        voltageSuppliedSpindexer = hopperSpindexerMotor.getMotorVoltage();

        statorCurrentRoller = hopperRollerMotor.getStatorCurrent();
        supplyCurrentRoller = hopperRollerMotor.getSupplyCurrent();
        tempRoller = hopperRollerMotor.getDeviceTemp();
        voltageSuppliedRoller = hopperRollerMotor.getMotorVoltage();

        Phoenix6Util.registerSignals(
            false,
            statorCurrentSpindexer,
            supplyCurrentSpindexer,
            tempSpindexer,
            voltageSuppliedSpindexer,

            statorCurrentRoller,
            supplyCurrentRoller,
            tempRoller,
            voltageSuppliedRoller);


        configHopperSpindexerMotor(hopperSpindexerMotor);
        configHopperRollerMotor(hopperRollerMotor);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.spindexerMotorConnected =
            connectedSpindexerDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    supplyCurrentSpindexer,
                    statorCurrentSpindexer,
                    tempSpindexer,
                    voltageSuppliedSpindexer
                )); 

        inputs.rollerMotorConnected =
            connectedRollerDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    supplyCurrentRoller,
                    statorCurrentRoller,
                    tempRoller,
                    voltageSuppliedRoller
                ));

        inputs.spindexerStatorCurrentAmps = statorCurrentSpindexer.getValue();
        inputs.spindexerSupplyCurrentAmps = supplyCurrentSpindexer.getValue();
        inputs.spindexerTemperatureCelsius = tempSpindexer.getValue();
        inputs.spindexerVoltageSupplied = voltageSuppliedSpindexer.getValue();

        inputs.rollerStatorCurrentAmps = statorCurrentRoller.getValue();
        inputs.rollerSupplyCurrentAmps = supplyCurrentRoller.getValue();
        inputs.rollerTemperatureCelsius = tempRoller.getValue();
        inputs.rollerVoltageSupplied = voltageSuppliedRoller.getValue();
    }

    @Override
    public void setSpindexerVoltage(double volts) {
        this.hopperSpindexerMotor.setControl(hopperSpindexerVoltageRequest.withOutput(volts));
    }

    @Override
    public void setRollerVoltage(double volts) {
        this.hopperRollerMotor.setControl(hopperRollerVoltageRequest.withOutput(volts));
    }

    private void configHopperSpindexerMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT;

        config.Feedback.SensorToMechanismRatio = SPINDEXER_GEAR_RATIO;

        config.MotorOutput.Inverted = 
            SPINDEXER_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperSpindexerConfigAlert);

        FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper spindexer motor", motor);
    }

    private void configHopperRollerMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;

        config.Feedback.SensorToMechanismRatio = SPINDEXER_GEAR_RATIO;

        config.MotorOutput.Inverted = 
            ROLLER_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperRollerConfigAlert);

        FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper roller motor", motor);
    }


}
