package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

// Note: the following includes code for a possible hopper Roller motor, all commented out.
// This was written when it was believed by the author that the subsystem included that motor.
// That code will stay here, commented out, while it is still possible that that motor makes a return.

public class HopperIOTalonFX implements HopperIO {
    private TalonFX hopperSpindexerMotor;
    private TalonFX hopperRollerMotor;

    private VoltageOut hopperSpindexerVoltageRequest;
    //private VoltageOut hopperRollerVoltageRequest;

    private MotionMagicVelocityVoltage hopperSpindexerVelocityRequest;
    
    private StatusSignal<Voltage> voltageSuppliedSpindexerStatusSignal;
    //private StatusSignal<Voltage> voltageSuppliedRollerStatusSignal;

    private StatusSignal<Current> supplyCurrentSpindexerStatusSignal;
    //private StatusSignal<Current> supplyCurrentRollerStatusSignal;

    private StatusSignal<Current> statorCurrentSpindexerStatusSignal;
    //private StatusSignal<Current> statorCurrentRollerStatusSignal;

    private StatusSignal<Temperature> tempSpindexerStatusSignal;
    //private StatusSignal<Temperature> tempRollerStatusSignal;

    private StatusSignal<AngularVelocity> velocitySpindexerStatusSignal;

    private Alert hopperSpindexerConfigAlert =
      new Alert("Failed to apply configuration for hopper spindexer.", AlertType.kError);

    
    //private Alert hopperRollerConfigAlert =
     // new Alert("Failed to apply configuration for hopper roller.", AlertType.kError);
    
    private final Debouncer connectedSpindexerDebouncer = new Debouncer(0.5);
    //private final Debouncer connectedRollerDebouncer = new Debouncer(0.5);

    public HopperIOTalonFX() {
        hopperSpindexerMotor = new TalonFX(HOPPER_SPINDEXER_ID, RobotConfig.getInstance().getCANBus());
        //hopperRollerMotor = new TalonFX(HOPPER_ROLLER_ID, RobotConfig.getInstance().getCANBus());

        hopperSpindexerVoltageRequest = new VoltageOut(0.0);
        //hopperRollerVoltageRequest = new VoltageOut(0.0);

        statorCurrentSpindexerStatusSignal = hopperSpindexerMotor.getStatorCurrent();
        supplyCurrentSpindexerStatusSignal = hopperSpindexerMotor.getSupplyCurrent();
        tempSpindexerStatusSignal = hopperSpindexerMotor.getDeviceTemp();
        voltageSuppliedSpindexerStatusSignal = hopperSpindexerMotor.getMotorVoltage();
        velocitySpindexerStatusSignal = hopperSpindexerMotor.getVelocity();
        
        //statorCurrentRollerStatusSignal = hopperRollerMotor.getStatorCurrent();
        //supplyCurrentRollerStatusSignal = hopperRollerMotor.getSupplyCurrent();
        //tempRollerStatusSignal = hopperRollerMotor.getDeviceTemp();
        //voltageSuppliedRollerStatusSignal = hopperRollerMotor.getMotorVoltage();
        

        Phoenix6Util.registerSignals(
            false,
            statorCurrentSpindexerStatusSignal,
            supplyCurrentSpindexerStatusSignal,
            tempSpindexerStatusSignal,
            voltageSuppliedSpindexerStatusSignal,
            velocitySpindexerStatusSignal//,

           
            // statorCurrentRollerStatusSignal,
            // supplyCurrentRollerStatusSignal,
            // tempRollerStatusSignal,
            // voltageSuppliedRollerStatusSignal
            );
            

        configHopperSpindexerMotor(hopperSpindexerMotor);
        //configHopperRollerMotor(hopperRollerMotor);
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
                    velocitySpindexerStatusSignal
                )); 

        
        // inputs.rollerMotorConnected =
        //     connectedRollerDebouncer.calculate(
        //         BaseStatusSignal.isAllGood(
        //             supplyCurrentRollerStatusSignal,
        //             statorCurrentRollerStatusSignal,
        //             tempRollerStatusSignal,
        //             voltageSuppliedRollerStatusSignal
        //         ));
        
        inputs.spindexerStatorCurrentAmps = statorCurrentSpindexerStatusSignal.getValue();
        inputs.spindexerSupplyCurrentAmps = supplyCurrentSpindexerStatusSignal.getValue();
        inputs.spindexerTemperatureCelsius = tempSpindexerStatusSignal.getValue();
        inputs.spindexerVoltageSupplied = voltageSuppliedSpindexerStatusSignal.getValue();
        inputs.spindexerVelocity = velocitySpindexerStatusSignal.getValue();
        
        // inputs.rollerStatorCurrentAmps = statorCurrentRollerStatusSignal.getValue();
        // inputs.rollerSupplyCurrentAmps = supplyCurrentRollerStatusSignal.getValue();
        // inputs.rollerTemperatureCelsius = tempRollerStatusSignal.getValue();
        // inputs.rollerVoltageSupplied = voltageSuppliedRollerStatusSignal.getValue();
        
    }

    @Override
    public void setSpindexerVoltage(double volts) {
        this.hopperSpindexerMotor.setControl(hopperSpindexerVoltageRequest.withOutput(volts));
    }

       
    
    // @Override
    // public void setRollerVoltage(double volts) {
    //     this.hopperRollerMotor.setControl(hopperRollerVoltageRequest.withOutput(volts));
    // }
    
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
    
    // private void configHopperRollerMotor(TalonFX motor) {
    //     TalonFXConfiguration config = new TalonFXConfiguration();

    //     config.CurrentLimits.SupplyCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;
    //     config.CurrentLimits.SupplyCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;
    //     config.CurrentLimits.SupplyCurrentLowerTime = 0;
    //     config.CurrentLimits.SupplyCurrentLimitEnable = true;
    //     config.CurrentLimits.StatorCurrentLimit = ROLLER_MOTOR_PEAK_CURRENT_LIMIT;

    //     config.Feedback.SensorToMechanismRatio = SPINDEXER_GEAR_RATIO;

    //     config.MotorOutput.Inverted = 
    //         ROLLER_MOTOR_INVERTED
    //             ? InvertedValue.Clockwise_Positive
    //             : InvertedValue.CounterClockwise_Positive;
    //     config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //     Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperRollerConfigAlert);

    //     FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper roller motor", motor);
    //}
    

}
