package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int FLYWHEEL_LEAD_MOTOR_ID = 20;
  public static final int FLYWHEEL_FOLLOW_1_MOTOR_ID = 21;
  public static final int FLYWHEEL_FOLLOW_2_MOTOR_ID = 22;
  public static final int HOOD_MOTOR_ID = 23;
  public static final int TURRET_MOTOR_ID = 26;

  public static final double FLYWHEEL_KP = 8.0;
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;
  public static final double FLYWHEEL_KS = 5.3;
  public static final double FLYWHEEL_KV = 0.04;
  public static final double FLYWHEEL_KA = 0.0;

  public static final boolean FLYWHEEL_LEAD_INVERTED = true;
  public static final double FLYWHEEL_LEAD_GEAR_RATIO = 1.5;
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.1; // FIXME: update value

  public static final boolean FLYWHEEL_FOLLOWER_1_INVERTED_FROM_LEAD =
      true; // FIXME: change to opposed if reversed

  // Follow 2 Constants
  public static final boolean FLYWHEEL_FOLLOWER_2_INVERTED_FROM_LEAD =
      false; // FIXME: change to opposed if reversed

  public static final double TURRET_KP = 50.0;
  public static final double TURRET_KI = 0.0;
  public static final double TURRET_KD = 0.0;
  public static final double TURRET_KS = 0.4;
  public static final double TURRET_KV = 0.0;
  public static final double TURRET_KA = 0.0;

  public static final boolean TURRET_INVERTED = false;
  public static final double TURRET_GEAR_RATIO = 41.666; // FIXME: way too low ask Noga for correct
  public static final double TURRET_LENGTH_METERS =
      1.0; // this is irrelevant for simulation since gravity doesn't affect the model
  public static final double TURRET_MASS_KG = 1.0; // FIXME: update value

  public static final double HOOD_KP = 325;
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;
  public static final double HOOD_KS = 0.4;
  public static final double HOOD_KV = 0.0;
  public static final double HOOD_KA = 0.0;

  public static final boolean HOOD_INVERTED = false;
  public static final double HOOD_GEAR_RATIO = 144.0;
  public static final double HOOD_LENGTH_METERS = 0.2;
  public static final double HOOD_MASS_KG = 2.0;
  public static final Angle HOOD_MIN_ANGLE = Degrees.of(20.0);
  public static final Angle HOOD_MAX_ANGLE = Degrees.of(52.0);
  public static final Angle HOOD_STARTING_ANGLE = Degrees.of(20.0);

  public static final Angle HOOD_LOWER_ANGLE_LIMIT = Degrees.of(22.0);
  public static final Angle HOOD_UPPER_ANGLE_LIMIT = Degrees.of(50.0); // FIXME: update value

  public static final Voltage HOOD_SLOW_LOWER_VOLTAGE = Volts.of(-1.0); // FIXME: update value

  public static final Angle TURRET_LOWER_ANGLE_LIMIT = Degrees.of(-180);
  public static final Angle TURRET_UPPER_ANGLE_LIMIT = Degrees.of(90);

  public static final Angle TURRET_STARTING_ANGLE = Degrees.of(90.0);

  public static final int FLYWHEEL_PEAK_CURRENT_LIMIT = 80;
  public static final int TURRET_PEAK_CURRENT_LIMIT = 40;

  public static final double HOOD_PEAK_CURRENT_LIMIT = 20;
  public static final int HOOD_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double HOOD_PEAK_CURRENT_DURATION = 0.1;

  public static final int TURRET_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double TURRET_PEAK_CURRENT_DURATION = 0.1;

  public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(0.5);
  public static final Angle HOOD_TOLERANCE_ANGLE = Degrees.of(0.25); // FIXME: update when tuning
  public static final Angle TURRET_TOLERANCE_ANGLE = Degrees.of(0.25); // FIXME: update when tuning

  // Hood and turret position setpoints
  public static final double HOOD_SETPOINT_1_DEGREES = 0.0; // FIXME: determine test points
  public static final double HOOD_SETPOINT_2_DEGREES = 0.0; // FIXME: determine test points
  public static final double HOOD_SETPOINT_3_DEGREES = 0.0; // FIXME: determine test points

  public static final double TURRET_SETPOINT_1_DEGREES = 0.0; // FIXME: determine test points
  public static final double TURRET_SETPOINT_2_DEGREES = 0.0; // FIXME: determine test points
  public static final double TURRET_SETPOINT_3_DEGREES = 0.0; // FIXME: determine test points

  public static final Angle TURRET_LOCK_POSITION_DEGREES =
      Degrees.of(120); // FIXME: determine value

  // Velocity setpoints for lead flywheel
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_1_RPS =
      RotationsPerSecond.of(30); // FIXME: determine test points
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_2_RPS =
      RotationsPerSecond.of(60); // FIXME: determine test points
  public static final AngularVelocity FLYWHEEL_VELOCITY_SETPOINT_3_RPS =
      RotationsPerSecond.of(90); // FIXME: determine test points

  public static final double COMMAND_WAIT_TIME_SECONDS = 3;

  public static final double HOOD_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double HOOD_CURRENT_TIME_THRESHOLD_SECONDS = 0.1;

  public static final double TURRET_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double TURRET_CURRENT_TIME_THRESHOLD_SECONDS = 0.1;

  // Displacement of the turret relative to the center of the robot at the phi of robot = 0
  public static final Transform2d ROBOT_TO_TURRET_TRANSFORM =
      new Transform2d(-0.0635, 0.13335, new Rotation2d()); // got from CAD

  public static final Angle HOOD_OFFSET_WHEN_SHOOTING = Degrees.of(5); // FIXME: determine value

  public static final double FLYWHEEL_VELOCITY_SCALE_FACTOR = 0.9; // FIXME: determine value

  public static final AngularVelocity LOCK_SHOT_FLYWHEEL_RPS =
      RotationsPerSecond.of(20); // FIXME: determine value
  public static final Angle LOCK_SHOT_HOOD_ANGLE = Degrees.of(22);
  public static final Angle LOCK_SHOT_TURRET_ANGLE = Degrees.of(30); // FIXME: determine value
}
