package frc.robot.subsystems.Climber;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_CAN_ID = 17;

    public static final boolean CLIMBER_MOTOR_INVERTED = false;
    
    // PID constants
    public static final double KP_SLOT0 = 0.0;
    public static final double KI_SLOT0 = 0.0;
    public static final double KD_SLOT0 = 0.0;

    // feed forward constants
    public static final double KS_SLOT0 = 0.0;
    public static final double KV_SLOT0 = 0.0;
    public static final double KA_SLOT0 = 0.0;
    public static final double KG_SLOT0 = 0.0;

    // Motion magic constants
    public static final double KV_EXPO = 0.0;
    public static final double KA_EXPO = 0.0;
    public static final double CRUISE_VELOCITY = 0.0;

    // the following are determined based on the mechanical design of the climber
    public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 0.0;
    public static final double CLIMBER_STATOR_CURRENT_LIMIT = 0.0;
    public static final double CLIMBER_PEAK_CURRENT_LIMIT = 0.0;
    public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.0;
    public static final double GEAR_RATIO = 0.0;
    public static final double CLIMB_VOLTAGE = 0.0;
    public static final double RETRACT_VOLTAGE_SLOW = 0.0;
    public static final double CIRFUMFERENCE_INCHES = 0.0;
    public static final double MAX_HEIGHT_INCHES = 0.0;
}