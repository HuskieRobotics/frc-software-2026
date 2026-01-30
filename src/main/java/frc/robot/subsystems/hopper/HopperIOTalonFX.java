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
import frc.lib.team6328.util.LoggedTunableNumber;

// Note: the following includes code for a possible hopper Roller motor, all commented out.
// This was written when it was believed by the author that the subsystem included that motor.
// That code will stay here, commented out, while it is still possible that that motor makes a return.

public class HopperIOTalonFX implements HopperIO {
    private TalonFX hopperSpindexerMotor;
    private TalonFX hopperRollerMotor;

    private VoltageOut hopperSpindexerVoltageRequest;
    private VoltageOut hopperRollerVoltageRequest;

    private MotionMagicVelocityVoltage hopperSpindexerVelocityRequest;
    private MotionMagicVelocityVoltage hopperRollerVelocityRequest;
    
    private StatusSignal<Voltage> voltageSuppliedSpindexerStatusSignal;
    private StatusSignal<Voltage> voltageSuppliedRollerStatusSignal;

    private StatusSignal<Current> supplyCurrentSpindexerStatusSignal;
    private StatusSignal<Current> supplyCurrentRollerStatusSignal;

    private StatusSignal<Current> statorCurrentSpindexerStatusSignal;
    private StatusSignal<Current> statorCurrentRollerStatusSignal;

    private StatusSignal<Temperature> tempSpindexerStatusSignal;
    private StatusSignal<Temperature> tempRollerStatusSignal;

    private StatusSignal<AngularVelocity> velocitySpindexerStatusSignal;
    private StatusSignal<AngularVelocity> velocityRollerStatusSignal;

    private Alert hopperSpindexerConfigAlert =
      new Alert("Failed to apply configuration for hopper spindexer.", AlertType.kError);

    
    private Alert hopperRollerConfigAlert =
     new Alert("Failed to apply configuration for hopper roller.", AlertType.kError);

    private final LoggedTunableNumber spindexerMotorKP = new LoggedTunableNumber("Hopper/SPINDEXER_KP", HopperConstants.SPINDEXER_KP);
    private final LoggedTunableNumber spindexerMotorKI = new LoggedTunableNumber("Hopper/SPINDEXER_KI", HopperConstants.SPINDEXER_KI);
    private final LoggedTunableNumber spindexerMotorKD = new LoggedTunableNumber("Hopper/SPINDEXER_KD", HopperConstants.SPINDEXER_KD);
    private final LoggedTunableNumber spindexerMotorKV = new LoggedTunableNumber("Hopper/SPINDEXER_KV", HopperConstants.SPINDEXER_KV);

    private final LoggedTunableNumber rollerMotorKP = new LoggedTunableNumber("Hopper/ROLLER_KP", HopperConstants.ROLLER_KP);
    private final LoggedTunableNumber rollerMotorKI = new LoggedTunableNumber("Hopper/ROLLER_KI", HopperConstants.ROLLER_KI);
    private final LoggedTunableNumber rollerMotorKD = new LoggedTunableNumber("Hopper/ROLLER_KD", HopperConstants.ROLLER_KD);
    private final LoggedTunableNumber rollerMotorKV = new LoggedTunableNumber("Hopper/ROLLER_KV", HopperConstants.ROLLER_KV);
    
    private final Debouncer connectedSpindexerDebouncer = new Debouncer(0.5);
    private final Debouncer connectedRollerDebouncer = new Debouncer(0.5);

    public HopperIOTalonFX() {
        hopperSpindexerMotor = new TalonFX(HOPPER_SPINDEXER_ID, RobotConfig.getInstance().getCANBus());
        hopperRollerMotor = new TalonFX(HOPPER_ROLLER_ID, RobotConfig.getInstance().getCANBus());

        hopperSpindexerVoltageRequest = new VoltageOut(0.0);
        hopperRollerVoltageRequest = new VoltageOut(0.0);

        statorCurrentSpindexerStatusSignal = hopperSpindexerMotor.getStatorCurrent();
        supplyCurrentSpindexerStatusSignal = hopperSpindexerMotor.getSupplyCurrent();
        tempSpindexerStatusSignal = hopperSpindexerMotor.getDeviceTemp();
        voltageSuppliedSpindexerStatusSignal = hopperSpindexerMotor.getMotorVoltage();
        velocitySpindexerStatusSignal = hopperSpindexerMotor.getVelocity();
        
        statorCurrentRollerStatusSignal = hopperRollerMotor.getStatorCurrent();
        supplyCurrentRollerStatusSignal = hopperRollerMotor.getSupplyCurrent();
        tempRollerStatusSignal = hopperRollerMotor.getDeviceTemp();
        voltageSuppliedRollerStatusSignal = hopperRollerMotor.getMotorVoltage();
        velocityRollerStatusSignal = hopperRollerMotor.getVelocity();
        
        hopperSpindexerVelocityRequest = new MotionMagicVelocityVoltage(0.0);
        hopperRollerVelocityRequest = new MotionMagicVelocityVoltage(0.0);

        Phoenix6Util.registerSignals(
            false,
            statorCurrentSpindexerStatusSignal,
            supplyCurrentSpindexerStatusSignal,
            tempSpindexerStatusSignal,
            voltageSuppliedSpindexerStatusSignal,
            velocitySpindexerStatusSignal,

           
            statorCurrentRollerStatusSignal,
            supplyCurrentRollerStatusSignal,
            tempRollerStatusSignal,
            voltageSuppliedRollerStatusSignal,
            velocityRollerStatusSignal
            );
            

        configHopperSpindexerMotor(hopperSpindexerMotor);
        configHopperRollerMotor(hopperRollerMotor);
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

        
        inputs.rollerMotorConnected =
            connectedRollerDebouncer.calculate(
                 BaseStatusSignal.isAllGood(
                    supplyCurrentRollerStatusSignal,
                    statorCurrentRollerStatusSignal,
                    tempRollerStatusSignal,
                    voltageSuppliedRollerStatusSignal,
                    velocityRollerStatusSignal
                 ));
        
        inputs.spindexerStatorCurrentAmps = statorCurrentSpindexerStatusSignal.getValue();
        inputs.spindexerSupplyCurrentAmps = supplyCurrentSpindexerStatusSignal.getValue();
        inputs.spindexerTemperatureCelsius = tempSpindexerStatusSignal.getValue();
        inputs.spindexerVoltageSupplied = voltageSuppliedSpindexerStatusSignal.getValue();
        inputs.spindexerVelocity = velocitySpindexerStatusSignal.getValue();
        
        inputs.rollerStatorCurrentAmps = statorCurrentRollerStatusSignal.getValue();
        inputs.rollerSupplyCurrentAmps = supplyCurrentRollerStatusSignal.getValue();
        inputs.rollerTemperatureCelsius = tempRollerStatusSignal.getValue();
        inputs.rollerVoltageSupplied = voltageSuppliedRollerStatusSignal.getValue();
        inputs.rollerVelocity = velocityRollerStatusSignal.getValue();
        
    }

    @Override
    public void setSpindexerVoltage(double volts) {
        this.hopperSpindexerMotor.setControl(hopperSpindexerVoltageRequest.withOutput(volts));
    }

       
    
    @Override
    public void setRollerVoltage(double volts) {
        this.hopperRollerMotor.setControl(hopperRollerVoltageRequest.withOutput(volts));
    }

    @Override
    public void setSpindexerVelocity(AngularVelocity velocity) {
        hopperSpindexerMotor.setControl(hopperSpindexerVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        hopperRollerMotor.setControl(hopperRollerVelocityRequest.withVelocity(velocity));
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

        config.Slot0.kP = spindexerMotorKP.get();
        config.Slot0.kI = spindexerMotorKI.get();
        config.Slot0.kD = spindexerMotorKD.get();
        config.Slot0.kV = spindexerMotorKV.get();

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

         config.Slot0.kP = rollerMotorKP.get();
         config.Slot0.kI = rollerMotorKI.get();
         config.Slot0.kD = rollerMotorKD.get();
         config.Slot0.kV = rollerMotorKV.get();

         Phoenix6Util.applyAndCheckConfiguration(motor, config, hopperRollerConfigAlert);

         FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "hopper roller motor", motor);
    }
    

}
