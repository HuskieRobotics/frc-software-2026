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
  public static final int FLYWHEEL_LEAD_MOTOR_ID = 9; // Updated in robot devices and trackers sheet
  public static final int FLYWHEEL_FOLLOW_1_MOTOR_ID = 21;
  // FIXME: update from robot devices and trackers sheet
  public static final int FLYWHEEL_FOLLOW_2_MOTOR_ID = 22;
  public static final int HOOD_MOTOR_ID = 23;
  public static final int TURRET_MOTOR_ID = 14;

  // Flywheel PID Constants (subject to change)
  public static final double FLYWHEEL_LEAD_ROTATION_KP = 19; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KI = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KD = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KS = 0.39217;
  public static final double FLYWHEEL_LEAD_ROTATION_KV = 0.086186; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KA = 0.022794; // FIXME: update value

  public static final boolean FLYWHEEL_LEAD_INVERTED = false;
  public static final double FLYWHEEL_LEAD_GEAR_RATIO = 1.0;
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.1; // FIXME: update value

  // Follow 1 Constants
  public static final boolean FLYWHEEL_FOLLOW_1_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_1_GEAR_RATIO = 1.0;

  // Follow 2 Constants
  public static final boolean FLYWHEEL_FOLLOW_2_INVERTED = false;
  public static final double FLYWHEEL_FOLLOW_2_GEAR_RATIO = 1.0;

  // Turret PID Constants (subject to change)
  public static final double TURRET_ROTATION_KP = 19; // FIXME: update value
  public static final double TURRET_ROTATION_KI = 0.0; // FIXME: update value
  public static final double TURRET_ROTATION_KD = 0.0; // FIXME: update value
  public static final double TURRET_ROTATION_KV = 0.12; // FIXME: update value
  public static final double TURRET_ROTATION_KA = 0.022794; // FIXME: update value

  // Turret motion magic pid constants
  public static final double TURRET_ROTATION_EXPO_KV = 5;
  public static final double TURRET_ROTATION_EXPO_KA = 5;
  public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY = 5;

  public static final boolean TURRET_INVERTED = false;
  public static final double TURRET_GEAR_RATIO = 15;
  public static final double TURRET_LENGTH_METERS =
      1.0; // this doesn't is irrelevant for the simulation since gravity doesn't affect the model
  public static final double TURRET_MASS_KG = 1.0; // FIXME: update value

  // Hood PID Constants (subject to change)
  public static final double HOOD_ROTATION_KP = 400;
  public static final double HOOD_ROTATION_KI = 0.0;
  public static final double HOOD_ROTATION_KD = 0.0;
  public static final double HOOD_ROTATION_KG = 0.0;
  public static final double HOOD_ROTATION_KS = 0.50489;
  public static final double HOOD_ROTATION_KV = 21.619;
  public static final double HOOD_ROTATION_KA = 0.83438;

  public static final double HOOD_ROTATION_EXPO_KV = 0.2342;
  public static final double HOOD_ROTATION_EXPO_KA = 0.20905;
  public static final double HOOD_MOTION_MAGIC_CRUISE_VELOCITY = 0.0;

  //  Hood physical constants
  public static final boolean HOOD_INVERTED = false;
  public static final double HOOD_GEAR_RATIO = 144.0;
  public static final double HOOD_LENGTH_METERS = 0.2;
  public static final double HOOD_MASS_KG = 2.0;
  public static final double HOOD_MIN_ANGLE = 52.0; // IFIXME: Is this even necessary?
  public static final double HOOD_MAX_ANGLE = 90;
  public static final double HOOD_STARTING_ANGLE = 20.0;

  // Hood angle limits
  public static final Angle HOOD_LOWER_ANGLE_LIMIT = Degrees.of(22.0); // FIXME: update value
  public static final Angle HOOD_UPPER_ANGLE_LIMIT = Degrees.of(50.0); // FIXME: update value
  public static final Angle HOOD_TOLERANCE_ANGLE = Degrees.of(1.5);

  // Turret angle limits
  public static final Angle TURRET_LOWER_ANGLE_LIMIT = Degrees.of(0); // FIXME: update value
  public static final Angle TURRET_UPPER_ANGLE_LIMIT = Degrees.of(90);
  public static final Angle TURRET_TOLERANCE_ANGLE = Degrees.of(1.5);

  // Current limits
  public static final int FLYWHEEL_PEAK_CURRENT_LIMIT = 40;
  public static final int TURRET_PEAK_CURRENT_LIMIT = 20;
  public static final double HOOD_PEAK_CURRENT_LIMIT = 20.0;
  public static final int HOOD_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double HOOD_PEAK_CURRENT_DURATION = 0.1;
  public static final int TURRET_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double TURRET_PEAK_CURRENT_DURATION = 0.1;

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
