package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  /* =======================
   *  CAN IDs
   * ======================= */
  public static final int FLYWHEEL_LEAD_MOTOR_ID = 9; // Updated in robot devices and trackers sheet
  public static final int FLYWHEEL_FOLLOW_1_MOTOR_ID =
      21; // FIXME: update from robot devices and trackers sheet
  public static final int FLYWHEEL_FOLLOW_2_MOTOR_ID = 22;
  public static final int HOOD_MOTOR_ID = 23;
  public static final int TURRET_MOTOR_ID = 14;

  /* =======================
   *  Flywheel PID Constants
   * ======================= */
  public static final double FLYWHEEL_LEAD_ROTATION_KP = 19; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KI = 0.0; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KD = 0.0; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KS = 0.39217; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KV = 0.086186; // FIXME: update value
  public static final double FLYWHEEL_LEAD_ROTATION_KA = 0.022794; // FIXME: update value

  public static final boolean FLYWHEEL_LEAD_INVERTED = true;
  public static final double FLYWHEEL_LEAD_GEAR_RATIO =
      2; // FIXME: Currently 2:1, could be 3:2 or 1:1
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.1; // FIXME: update value

  public static final MotorAlignmentValue FLYWHEEL_FOLLOW_1_INVERTED_FROM_LEAD =
      MotorAlignmentValue.Aligned; // FIXME: change to opposed if reversed

  // Follow 2 Constants
  public static final MotorAlignmentValue FLYWHEEL_FOLLOW_2_INVERTED_FROM_LEAD =
      MotorAlignmentValue.Aligned; // FIXME: change to opposed if reversed

  /* =======================
   *  Turret PID Constants
   * ======================= */
  public static final double TURRET_ROTATION_KP = 19; // FIXME: update value
  public static final double TURRET_ROTATION_KI = 0.0; // FIXME: update value
  public static final double TURRET_ROTATION_KD = 0.0; // FIXME: update value
  public static final double TURRET_ROTATION_KS = 0.39217; // FIXME: update value
  public static final double TURRET_ROTATION_KV = 0.12; // FIXME: update value
  public static final double TURRET_ROTATION_KA = 0.022794; // FIXME: update value

  /* =======================
   *  Turret Motion Magic Constants
   * ======================= */
  public static final double TURRET_ROTATION_EXPO_KV = 5;
  public static final double TURRET_ROTATION_EXPO_KA = 5; // FIXME: update values when tuning
  public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY =
      5; // FIXME: update values when tuning

  public static final boolean TURRET_INVERTED =
      false; // FIXME: update when first running the turret
  public static final double TURRET_GEAR_RATIO = 41.666; // FIXME: way too low ask Noga for correct
  public static final double TURRET_LENGTH_METERS =
      1.0; // this is irrelevant for simulation since gravity doesn't affect the model
  public static final double TURRET_MASS_KG = 1.0; // FIXME: update value

  /* =======================
   *  Hood PID Constants
   * ======================= */
  public static final double HOOD_ROTATION_KP = 400; // FIXME: update values when tuning
  public static final double HOOD_ROTATION_KI = 0.0; // FIXME: update values when tuning
  public static final double HOOD_ROTATION_KD = 0.0; // FIXME: update values when tuning
  public static final double HOOD_ROTATION_KS = 0.50489; // FIXME: update values when tuning
  public static final double HOOD_ROTATION_KV = 21.619; // FIXME: update values when tuning
  public static final double HOOD_ROTATION_KA = 0.83438; // FIXME: update values when tuning

  /* =======================
   *  Hood Physical Constants
   * ======================= */
  public static final boolean HOOD_INVERTED = false;
  public static final double HOOD_GEAR_RATIO = 144.0;
  public static final double HOOD_LENGTH_METERS = 0.2;
  public static final double HOOD_MASS_KG = 2.0;
  public static final Angle HOOD_MIN_ANGLE = Degrees.of(12.0);
  public static final Angle HOOD_MAX_ANGLE = Degrees.of(52);
  public static final Angle HOOD_STARTING_ANGLE = Degrees.of(20.0);

  /* =======================
   *  Hood Angle Limits
   * ======================= */
  public static final Angle HOOD_LOWER_ANGLE_LIMIT = Degrees.of(22); // FIXME: update value
  public static final Angle HOOD_UPPER_ANGLE_LIMIT = Degrees.of(50); // FIXME: update value

  /* =======================
   *  Turret Angle Limits
   * ======================= */

  public static final Angle TURRET_LOWER_ANGLE_LIMIT = Degrees.of(0); // FIXME: update value
  public static final Angle TURRET_UPPER_ANGLE_LIMIT = Degrees.of(360); // FIXME: update value

  /* =======================
   *  Current Limits
   * ======================= */
  public static final int FLYWHEEL_PEAK_CURRENT_LIMIT = 80;
  public static final int TURRET_PEAK_CURRENT_LIMIT = 20;

  public static final double HOOD_PEAK_CURRENT_LIMIT = 20.0;
  public static final int HOOD_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double HOOD_PEAK_CURRENT_DURATION = 0.1;

  public static final int TURRET_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double TURRET_PEAK_CURRENT_DURATION = 0.1;

  /* =======================
   *  Tolerances
   * ======================= */
  public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(0.5);
  public static final Angle HOOD_TOLERANCE_ANGLE = Degrees.of(0.25); // FIXME: update when tuning
  public static final Angle TURRET_TOLERANCE_ANGLE = Degrees.of(0.25); // FIXME: update when tuning

  /* =======================
   *  Setpoints
   * ======================= */
  // Hood and turret position setpoints
  public static final double HOOD_SETPOINT_1_DEGREES = 0.0; // FIXME: determine test points
  public static final double HOOD_SETPOINT_2_DEGREES = 0.0; // FIXME: determine test points
  public static final double HOOD_SETPOINT_3_DEGREES = 0.0; // FIXME: determine test points

  public static final double TURRET_SETPOINT_1_DEGREES = 0.0; // FIXME: determine test points
  public static final double TURRET_SETPOINT_2_DEGREES = 0.0; // FIXME: determine test points
  public static final double TURRET_SETPOINT_3_DEGREES = 0.0; // FIXME: determine test points

  // Velocity setpoints for lead flywheel
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_1_RPS =
      RotationsPerSecond.of(30); // FIXME: determine test points
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_2_RPS =
      RotationsPerSecond.of(60); // FIXME: determine test points
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_3_RPS =
      RotationsPerSecond.of(90); // FIXME: determine test points

  /* =======================
   *  Timing & Jam Detection
   * ======================= */
  public static final double COMMAND_WAIT_TIME_SECONDS = 3;

  public static final double HOOD_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double HOOD_CURRENT_TIME_THRESHOLD_SECONDS = 0.1;

  public static final double TURRET_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double TURRET_CURRENT_TIME_THRESHOLD_SECONDS = 0.1;
}
