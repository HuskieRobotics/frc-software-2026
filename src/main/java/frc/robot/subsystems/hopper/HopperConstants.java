package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class HopperConstants {

  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final int SPINDEXER_ID = 14;
  public static final int KICKER_ID = 19;

  public static final double SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value
  public static final double KICKER_MOTOR_PEAK_CURRENT_LIMIT = 40; // FIXME: Update value

  public static final double SPINDEXER_GEAR_RATIO = 10.0;
  public static final double KICKER_GEAR_RATIO = 1.0;

  public static final boolean SPINDEXER_MOTOR_INVERTED = false;
  public static final boolean KICKER_MOTOR_INVERTED = false;

  public static final double SPINDEXER_MOTOR_MANUAL_CONTROL_VELOCITY = 1.0; // FIXME: Update value?
  public static final double KICKER_MOTOR_MANUAL_CONTROL_VELOCITY = 1.0; // FIXME: Update value?

  public static final double SPINDEXER_KP = 12.0; // FIXME: Update value
  public static final double SPINDEXER_KI = 0.0; // FIXME: Update value
  public static final double SPINDEXER_KD = 0.0; // FIXME: Update value
  public static final double SPINDEXER_KV = 0.67505; // FIXME: Update value
  public static final double SPINDEXER_KA = 0.027564; // FIXME: Update value
  public static final double SPINDEXER_KS = 0.01; // FIXME: Update value

  public static final AngularVelocity SPINDEXER_VELOCITY_TOLERANCE =
      RotationsPerSecond.of(0.5); // FIXME: Update value

  public static final double KICKER_KP = 12.0; // FIXME: Update value
  public static final double KICKER_KI = 0.0; // FIXME: Update value
  public static final double KICKER_KD = 0.0; // FIXME: Update vlaue
  public static final double KICKER_KV = 0.67505; // FIXME: Update value
  public static final double KICKER_KA = 0.027564; // FIXME: Update value
  public static final double KICKER_KS = 0.01; // FIXME: Update value
  public static final double KICKER_MOI = 0.5; // FIXME: Update value

  public static final AngularVelocity KICKER_VELOCITY_TOLERANCE =
      RotationsPerSecond.of(5); // FIXME: Update value

  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS = 0.0; // FIXME: Update value
  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.0; // FIXME: Update value
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_AMPS = 0.0; // FIXME: Update value
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_SECONDS = 0.0; // FIXME: Update value

  public static final AngularVelocity SPINDEXER_UNJAM_VELOCITY =
      RotationsPerSecond.of(
          -5); // <-- Should be negative (rotations per second) // FIXME: Update value
  public static final AngularVelocity KICKER_UNJAM_VELOCITY =
      RotationsPerSecond.of(
          -5); // <-- Should be negative (rotations per second) // FIXME: Update value\

  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_1_RPS = RotationsPerSecond.of(30); // FIXME: Determine test points
  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_2_RPS = RotationsPerSecond.of(60); // FIXME: Determine test points
  public static final AngularVelocity KICKER_VELOCITY_SETPOINT_3_RPS = RotationsPerSecond.of(90); // FIXME: Determine test points

  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_1_RPS = RotationsPerSecond.of(30); // FIXME: Determine test points
  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_2_RPS = RotationsPerSecond.of(60); // FIXME: Determine test points
  public static final AngularVelocity SPINDEXER_VELOCITY_SETPOINT_3_RPS = RotationsPerSecond.of(90); // FIXME: Determine test points
}
