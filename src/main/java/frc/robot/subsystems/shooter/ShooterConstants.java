package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";
  public static final boolean TESTING = false;

  // change the ids once we get the updated stuff
  public static final int FLYWHEEL_LEAD_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW_1_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW_2_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW_3_MOTOR_ID = 0;
  public static final int HOOD_MOTOR_ID = 0;
  public static final int TURRET_MOTOR_ID = 0;
  public static final int KICKER_MOTOR_ID = 0;

  // Flywheel PID Constants (subject to change)
  public static final double FLYWHEEL_LEAD_ROTATION_KP = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KI = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KD = 0.0;

  public static final boolean FLYWHEEL_LEAD_INVERTED = false;
  public static final double FLYWHEEL_LEAD_GEAR_RATIO = 1.0;

  // Follow 1 Constants
  public static final boolean FLYWHEEL_FOLLOW_1_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_1_GEAR_RATIO = 1.0;

  // Follow 2 Constants
  public static final boolean FLYWHEEL_FOLLOW_2_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_2_GEAR_RATIO = 1.0;

  // Follow 3 Constants
  public static final boolean FLYWHEEL_FOLLOW_3_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_3_GEAR_RATIO = 1.0;

  // Turret PID Constants (subject to change)
  public static final double TURRET_ROTATION_KP = 0.0;
  public static final double TURRET_ROTATION_KI = 0.0;
  public static final double TURRET_ROTATION_KD = 0.0;
  public static final double TURRET_ROTATION_EXPO_KV = 0.0;
  public static final double TURRET_ROTATION_EXPO_KA = 0.0;
  public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  public static final boolean TURRET_INVERTED = false;
  public static final double TURRET_GEAR_RATIO = 0; // FIXME: Update once we know

  // Hood PID Constants (subject to change)
  public static final double HOOD_ROTATION_KP = 0.0;
  public static final double HOOD_ROTATION_KI = 0.0;
  public static final double HOOD_ROTATION_KD = 0.0;
  public static final double HOOD_ROTATION_EXPO_KV = 0.0;
  public static final double HOOD_ROTATION_EXPO_KA = 0.0;
  public static final double HOOD_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  public static final boolean HOOD_INVERTED = false;
  public static final double HOOD_GEAR_RATIO = 0; // FIXME: Update once we know

  // Kicker PID Constants (subject to change)
  public static final double KICKER_ROTATION_KP = 0.0;
  public static final double KICKER_ROTATION_KI = 0.0;
  public static final double KICKER_ROTATION_KD = 0.0;

  public static final boolean KICKER_INVERTED = false;
  public static final double KICKER_GEAR_RATIO = 0; // FIXME: Update once we know

  // FIXME: to be deleted
  // public static final double MOTOR_KS = 0.0;
  // public static final double MOTOR_KG = 0.0;
  // public static final double ROTATION_KV = 0.0;
  // public static final double ROTATION_KA = 0.0;

  // Current limits
  public static final int FLYWHEEL_LEAD_PEAK_CURRENT_LIMIT = 40;
  public static final int TURRET_PEAK_CURRENT_LIMIT = 20;
  public static final int KICKER_PEAK_CURRENT_LIMIT = 20;
  public static final int HOOD_PEAK_CURRENT_LIMIT = 20;

  // Hard coded non-tunable physics values
  public static final double MOTOR_KS = 0.0;
  public static final double MOTOR_KG = 0.0;
  public static final double ROTATION_KV = 0.0;
  public static final double ROTATION_KA = 0.0;

  // Motion magic PID constants
  public static final double ROTATION_EXPO_KV = 0.0;
}
