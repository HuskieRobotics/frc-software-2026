package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class HopperConstants {

  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final int SPINDEXER_ID = 14;
  public static final int KICKER_ID = 19;

  public static final double SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value
  public static final double KICKER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value

  public static final double SPINDEXER_GEAR_RATIO = 18.0;
  public static final double KICKER_GEAR_RATIO = 1.0;

  public static final boolean SPINDEXER_MOTOR_INVERTED = true;
  public static final boolean KICKER_MOTOR_INVERTED = true;

  public static final double SPINDEXER_KP = 64.0;
  public static final double SPINDEXER_KI = 0.0;
  public static final double SPINDEXER_KD = 0.0;
  public static final double SPINDEXER_KV = 0.08;
  public static final double SPINDEXER_KA = 0.027564;
  public static final double SPINDEXER_KS = 2.5;

  public static final double SPINDEXER_MOI = 0.5;

  public static final AngularVelocity SPINDEXER_VELOCITY_TOLERANCE =
      RotationsPerSecond.of(0.5); // FIXME: Update value

  public static final double KICKER_KP = 11.0;
  public static final double KICKER_KI = 0.0;
  public static final double KICKER_KD = 0.0;
  public static final double KICKER_KV = 0.22;
  public static final double KICKER_KA = 0.0;
  public static final double KICKER_KS = 6.25;

  public static final double KICKER_MOI = 0.5;

  public static final AngularVelocity KICKER_VELOCITY_TOLERANCE =
      RotationsPerSecond.of(0.5); // FIXME: Update value

  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS = 39; // FIXME: Update value
  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.5;
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_AMPS = 39; // FIXME: Update value
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.5;

  public static final AngularVelocity SPINDEXER_UNJAM_VELOCITY =
      RotationsPerSecond.of(
          -5); // <-- Should be negative (rotations per second) // FIXME: Update value
  public static final AngularVelocity KICKER_UNJAM_VELOCITY =
      RotationsPerSecond.of(
          -5); // <-- Should be negative (rotations per second) // FIXME: Update value

  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_1_RPS =
      RotationsPerSecond.of(30); // FIXME: Determine test points
  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_2_RPS =
      RotationsPerSecond.of(60); // FIXME: Determine test points
  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_3_RPS =
      RotationsPerSecond.of(90); // FIXME: Determine test points

  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_1_RPS =
      RotationsPerSecond.of(30); // FIXME: Determine test points
  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_2_RPS =
      RotationsPerSecond.of(60); // FIXME: Determine test points
  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_3_RPS =
      RotationsPerSecond.of(90); // FIXME: Determine test points

  public static final double SPINDEXER_UNJAM_WAIT_TIME = 1.0; // FIXME: update value
  public static final double KICKER_UNJAM_WAIT_TIME = 1.0; // FIXME: update value

  public static final AngularVelocity KICKER_FUEL_INTO_SHOOTER_VELOCITY =
      RotationsPerSecond.of(20.0); // FIXME: Update value

  public static final AngularVelocity SPIN_FUEL_INTO_KICKER_VELOCITY =
      RotationsPerSecond.of(4.5); // FIXME: Update value
}
