package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  // change the ids once we get the updated stuff
  public static final int FLYWHEEL_LEAD_MOTOR_ID = 0; // Updated in robot devices and trackers sheet
  public static final int FLYWHEEL_FOLLOW_1_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW_2_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW_3_MOTOR_ID = 0;
  public static final int HOOD_MOTOR_ID = 0;
  public static final int TURRET_MOTOR_ID = 0;

  // Flywheel PID Constants (subject to change)
  public static final double FLYWHEEL_LEAD_ROTATION_KP = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KI = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KD = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KS = 0.0;

  public static final boolean FLYWHEEL_LEAD_INVERTED = false;
  public static final double FLYWHEEL_LEAD_GEAR_RATIO = 1.0;

  // Follow 1 Constants
  public static final boolean FLYWHEEL_FOLLOW_1_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_1_GEAR_RATIO = 1.0;

  // Follow 2 Constants
  public static final boolean FLYWHEEL_FOLLOW_2_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_2_GEAR_RATIO = 1.0;

  // Turret PID Constants (subject to change)
  public static final double TURRET_ROTATION_KP = 0.0;
  public static final double TURRET_ROTATION_KI = 0.0;
  public static final double TURRET_ROTATION_KD = 0.0;
  public static final double TURRET_ROTATION_KV = 0.0;
  public static final double TURRET_ROTATION_KA = 0.0;

// Turret motion magic pid constants
  public static final double TURRET_ROTATION_EXPO_KV = 0.0;
  public static final double TURRET_ROTATION_EXPO_KA = 0.0;
  public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  public static final boolean TURRET_INVERTED = false;
  public static final double TURRET_GEAR_RATIO = 0;

  // Hood PID Constants (subject to change)
  public static final double HOOD_ROTATION_KP = 0.0;
  public static final double HOOD_ROTATION_KI = 0.0;
  public static final double HOOD_ROTATION_KD = 0.0;
  public static final double HOOD_ROTATION_KG = 0.0;
  public static final double HOOD_ROTATION_KS = 0.0;
  public static final double HOOD_ROTATION_KV = 0.0;
  public static final double HOOD_ROTATION_KA = 0.0;

  public static final double HOOD_ROTATION_EXPO_KV = 0.0;
  public static final double HOOD_ROTATION_EXPO_KA = 0.0;
  public static final double HOOD_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  // SIM KV and KA values
  public static final double SIM_KV = 0.0;
  public static final double SIM_KA = 0.0;

  //  Hood physical constants
  public static final boolean HOOD_INVERTED = false;
  public static final double HOOD_GEAR_RATIO = 240;
  public static final double HOOD_LENGTH_METERS = 2;
  public static final double HOOD_MASS_KG = 5;
  public static final double HOOD_MIN_ANGLE_RAD = 0.0;
  public static final double HOOD_MAX_ANGLE_RAD = 90;
  public static final double HOOD_STARTING_ANGLE_RAD = 0.0;

  // Hood angle limits
  public static final Angle HOOD_UPPER_ANGLE_LIMIT = Degrees.of(0.0);
  public static final Angle HOOD_LOWER_ANGLE_LIMIT = Degrees.of(0.0);

  // Turret angle limits
  public static final Angle TURRET_UPPER_ANGLE_LIMIT = Degrees.of(0.0);
  public static final Angle TURRET_LOWER_ANGLE_LIMIT = Degrees.of(0.0);

  // Current limits
  public static final int FLYWHEEL_PEAK_CURRENT_LIMIT = 40;
  public static final int TURRET_PEAK_CURRENT_LIMIT = 20;
  public static final int HOOD_PEAK_CURRENT_LIMIT = 20;
  public static final int HOOD_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final int HOOD_PEAK_CURRENT_DURATION = 100;
  public static final int TURRET_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final int TURRET_PEAK_CURRENT_DURATION = 100;

  // Hard coded non-tunable physics values
  public static final double MOTOR_KS = 0.0;
  public static final double MOTOR_KG = 0.0;
  public static final double ROTATION_KV = 0.0;
  public static final double ROTATION_KA = 0.0;

  // Motion magic PID constants
  public static final double ROTATION_EXPO_KV = 0.0;

  public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(5.0);
  public static final double POSITION_TOLERANCE = 2.0;

  // Hood and turret position setpoints
  public static final double HOOD_SETPOINT_1_DEGREES = 0.0;
  public static final double HOOD_SETPOINT_2_DEGREES = 0.0;
  public static final double HOOD_SETPOINT_3_DEGREES = 0.0;

  public static final double TURRET_SETPOINT_1_DEGREES = 0.0;
  public static final double TURRET_SETPOINT_2_DEGREES = 0.0;
  public static final double TURRET_SETPOINT_3_DEGREES = 0.0;

  // Velocity setpoints for lead flywheel
  public static final double FLYWHEEL_VELOCITY_SETPOINT_1_RPS = 30;
  public static final double FLYWHEEL_VELOCITY_SETPOINT_2_RPS = 60;
  public static final double FLYWHEEL_VELOCITY_SETPOINT_3_RPS = 90;

  public static final double SHOOTER_IDLE_VELOCITY_RPS = 5; // idle velocity when not shooting

  public static final double COMMAND_WAIT_TIME_SECONDS = 3; // time to wait between commands
}
