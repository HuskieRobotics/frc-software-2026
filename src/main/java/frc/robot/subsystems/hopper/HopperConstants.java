package frc.robot.subsystems.hopper;

public class HopperConstants {

    // NOTE: Everything referring to the 'roller' motor is referred to as the 'kicker' motor by other subteams.
    // This is the motor that transfers/rolls/kicks fuel from the hopper into the shooter.
    // These are the same motor, we just have not changed the name of every variable/method.
    // READ THE ABOVE. !!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    public static final String SUBSYSTEM_NAME = "Hopper";

    public static final int HOPPER_SPINDEXER_ID = 70; // FIXME: Not at all correct
    public static final int HOPPER_ROLLER_ID = 71; // FIXME: Not at all correct

    public static final double SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value
    public static final double ROLLER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value

    public static final double SPINDEXER_GEAR_RATIO = 10.0; 
    public static final double ROLLER_GEAR_RATIO = 1.0; // FIXME: Update value

    public static final boolean SPINDEXER_MOTOR_INVERTED = false;
    public static final boolean ROLLER_MOTOR_INVERTED = false;

    public static final double SPINDEXER_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0; // FIXME: Update value?
    public static final double ROLLER_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0; // FIXME: Update value?

    public static final double SPINDEXER_KP = 0.0; // FIXME: Update value
    public static final double SPINDEXER_KI = 0.0; // FIXME: Update value
    public static final double SPINDEXER_KD = 0.0; // FIXME: Update value
    public static final double SPINDEXER_KV = 0.0; // FIXME: Update value

    public static final double ROLLER_KP = 0.0; // FIXME: Update value
    public static final double ROLLER_KI = 0.0; // FIXME: Update value
    public static final double ROLLER_KD = 0.0; // FIXME: Update vlaue
    public static final double ROLLER_KV = 0.0; // FIXME: Update value

    public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS = 0.0; // FIXME: Update value
    public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.0; // FIXME: Update value
    public static final double ROLLER_CURRENT_SPIKE_THRESHHOLD_AMPS = 0.0; // FIXME: Update value
    public static final double ROLLER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.0; // FIXME: Update value
}
