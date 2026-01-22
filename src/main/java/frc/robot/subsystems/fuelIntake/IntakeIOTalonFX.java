package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX rollerMotor;
    private TalonFX deployerMotor;


    //control output types
    private VoltageOut rollerVoltageRequest;
    private TorqueCurrentFOC deployerCurrentRequest;
    private DynamicMotionMagicExpoVoltage deployerPositionRequest;

    private Alert configAlert = 
        new Alert("Failed to apply configuration for Intake.", AlertType.kError);

    private final LoggedTunableNumber deployerKp =
        new LoggedTunableNumber("Intake/Deployer/kP", IntakeConstants.DEPLOYER_KP);
    private final LoggedTunableNumber deployerKi =
        new LoggedTunableNumber("Intake/Deployer/kI", IntakeConstants.DEPLOYER_KI);
    private final LoggedTunableNumber deployerKd =
        new LoggedTunableNumber("Intake/Deployer/kD", IntakeConstants.DEPLOYER_KD);
    private final LoggedTunableNumber deployerKs =
        new LoggedTunableNumber("Intake/Deployer/kS", IntakeConstants.DEPLOYER_KS);
    private final LoggedTunableNumber deployerKv =
        new LoggedTunableNumber("Intake/Deployer/kV", IntakeConstants.DEPLOYER_KV);
    private final LoggedTunableNumber deployerKa =
        new LoggedTunableNumber("Intake/Deployer/kA", IntakeConstants.DEPLOYER_KA);
    private final LoggedTunableNumber deployerKg =
        new LoggedTunableNumber("Intake/Deployer/kG", IntakeConstants.DEPLOYER_KG);

    private final LoggedTunableNumber deployerKvExpo =
        new LoggedTunableNumber("Intake/Deployer/kVExpo", IntakeConstants.DEPLOYER_KV_EXPO);
    private final LoggedTunableNumber deployerKaExpo =
        new LoggedTunableNumber("Intake/Deployer/kAExpo", IntakeConstants.DEPLOYER_KA_EXPO);
    private final LoggedTunableNumber deployerCruiseVelocity = 
        new LoggedTunableNumber("Intake/Deployer/cruiseVelocity", IntakeConstants.DEPLOYER_CRUISE_VELOCITY);

    private VelocitySystemSim rollerSim;
    private ArmSystemSim deployerSim;

    //roller status signals
    private StatusSignal<Voltage> rollerVoltageSS;
    private StatusSignal<AngularVelocity> rollerVelocitySS;
    private StatusSignal<Current> rollerStatorCurrentSS;
    private StatusSignal<Current> rollerSupplyCurrentSS;
    private StatusSignal<Temperature> rollerTempSS;

    //deployer status signals
    private StatusSignal<Voltage> deployerVoltageSS;
    private StatusSignal<Current> deployerStatorCurrentSS;
    private StatusSignal<Current> deployerSupplyCurrentSS;
    private StatusSignal<Temperature> deployerTempSS;
    private StatusSignal<Angle> deployerPositionSS;

    //debouncers
    private final Debouncer connectedRollerDebouncer = new Debouncer(0.5);
    private final Debouncer connectedDeployerDebouncer = new Debouncer(0.5);

    public IntakeIOTalonFX() {
        rollerMotor =
            new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
        deployerMotor =
            new TalonFX(IntakeConstants.DEPLOYER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
        
        rollerVoltageSS = rollerMotor.getMotorVoltage();
        deployerVoltageSS = deployerMotor.getMotorVoltage();

        rollerVelocitySS = rollerMotor.getVelocity();

        rollerStatorCurrentSS = rollerMotor.getStatorCurrent();
        deployerStatorCurrentSS = deployerMotor.getStatorCurrent();

        rollerSupplyCurrentSS = rollerMotor.getSupplyCurrent();
        deployerSupplyCurrentSS = deployerMotor.getSupplyCurrent();

        rollerTempSS = rollerMotor.getDeviceTemp();
        deployerTempSS = deployerMotor.getDeviceTemp();

        deployerPositionSS = deployerMotor.getPosition();

        Phoenix6Util.registerSignals(
            true,
            rollerVoltageSS,
            deployerVoltageSS,
            rollerVelocitySS,
            rollerStatorCurrentSS,
            deployerStatorCurrentSS,
            rollerSupplyCurrentSS,
            deployerSupplyCurrentSS,
            rollerTempSS,
            deployerTempSS,
            deployerPositionSS);

        rollerVoltageRequest = new VoltageOut(0);
        deployerCurrentRequest = new TorqueCurrentFOC(0);
        deployerPositionRequest = new DynamicMotionMagicExpoVoltage(0, deployerKvExpo.get(), deployerKaExpo.get());

        configDeployerMotor(deployerMotor);   
        configRollerMotor(rollerMotor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.rollerConnected = 
            connectedRollerDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    rollerVelocitySS,
                    rollerVoltageSS,
                    rollerStatorCurrentSS,
                    rollerTempSS,
                    rollerSupplyCurrentSS
                )
            );
        inputs.deployerConnected = 
            connectedDeployerDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    deployerVoltageSS,
                    deployerStatorCurrentSS,
                    deployerSupplyCurrentSS,
                    deployerTempSS,
                    deployerPositionSS
                )
            );

        inputs.rollerVelocityRPS = rollerVelocitySS.getValue();
        inputs.rollerStatorCurrentAmps = rollerStatorCurrentSS.getValue();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentSS.getValue();
        inputs.rollerTempCelcius = rollerTempSS.getValue();
        inputs.rollerVoltage = rollerVoltageSS.getValue();

        inputs.deployerVoltage = deployerVoltageSS.getValue();
        inputs.deployerStatorCurrentAmps = deployerStatorCurrentSS.getValue();
        inputs.deployerSupplyCurrentAmps = deployerSupplyCurrentSS.getValue();
        inputs.deployerTempCelcius = deployerTempSS.getValue();

        if(Constants.TUNING_MODE) {
            inputs.closedLoopError = 
                Rotations.of(deployerMotor.getClosedLoopError().getValueAsDouble());
            inputs.closedLoopReference = 
                Rotations.of(deployerMotor.getClosedLoopReference().getValueAsDouble());
        }

        inputs.angularPosition = deployerPositionSS.getValue();
        inputs.linearPosition = IntakeConstants.DEPLOYER_CIRCUIMFERENCE.times(inputs.angularPosition.in(Rotations));

        LoggedTunableNumber.ifChanged(
            hashCode(),
           motionMagic -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            this.deployerMotor.getConfigurator().refresh(config);
            config.Slot0.kP = motionMagic[0];
            config.Slot0.kI = motionMagic[1];
            config.Slot0.kD = motionMagic[2];
            config.Slot0.kS = motionMagic[3];
            config.Slot0.kV = motionMagic[4];
            config.Slot0.kA = motionMagic[5];
            config.Slot0.kG = motionMagic[6];

            config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
            config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];

            config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];

            this.deployerMotor.getConfigurator().apply(config);
           },
           deployerKp,
           deployerKi,
           deployerKd,
           deployerKs,
           deployerKv,
           deployerKa,
           deployerKg,
           deployerKvExpo,
           deployerKaExpo,
           deployerCruiseVelocity);


        deployerSim.updateSim();
        rollerSim.updateSim();
    }

    @Override
    public void setRollerVoltage(Voltage Volts) {
        this.rollerMotor.setControl(rollerVoltageRequest.withOutput(Volts));
    }

    @Override
    public void setDeployerCurrent(Current amps) {
        this.rollerMotor.setControl(deployerCurrentRequest);
    }

    @Override
    public void setDeployerPosition(Distance linearPosition){
        deployerMotor.setControl(
            deployerPositionRequest.withPosition(
                Rotations.of(linearPosition.div(IntakeConstants.DEPLOYER_CIRCUIMFERENCE).magnitude())
            )
        );
    }

    private void configDeployerMotor(TalonFX motor) {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOYER_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.DEPLOYER_LOWER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOYER_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = deployerKp.get();
        config.Slot0.kI = deployerKi.get();
        config.Slot0.kD = deployerKd.get();
        config.Slot0.kS = deployerKs.get();
        config.Slot0.kV = deployerKv.get();
        config.Slot0.kA = deployerKa.get();
        config.Slot0.kG = deployerKg.get();
        config.MotionMagic.MotionMagicExpo_kV = deployerKvExpo.get();
        config.MotionMagic.MotionMagicExpo_kA = deployerKaExpo.get();

        Phoenix6Util.applyAndCheckConfiguration(
            motor, config, configAlert);

        FaultReporter.getInstance().registerHardware(IntakeConstants.SUBSYSTEM_NAME, "Deployer Motor", motor);
    }

    private void configRollerMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.ROLLER_LOWER_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Phoenix6Util.applyAndCheckConfiguration(
            motor, config, configAlert);

        FaultReporter.getInstance().registerHardware(IntakeConstants.SUBSYSTEM_NAME, "Roller Motor", motor);
    }
    
}