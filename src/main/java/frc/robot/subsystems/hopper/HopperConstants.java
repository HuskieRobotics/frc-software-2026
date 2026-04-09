package frc.robot.subsystems.hopper;

public class HopperConstants {

  public static final String SUBSYSTEM_NAME = "Hopper";

  public static final int SPINDEXER_ID = 14;
  public static final int KICKER_ID = 19;

  public static final double SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT = 60;
  public static final double KICKER_MOTOR_PEAK_CURRENT_LIMIT = 60;

  public static final double SPINDEXER_GEAR_RATIO = 8.0;
  public static final double KICKER_GEAR_RATIO = 1.0;

  public static final boolean SPINDEXER_MOTOR_INVERTED = true;
  public static final boolean KICKER_MOTOR_INVERTED = true;

  public static final double SPINDEXER_KP = 80.0;
  public static final double SPINDEXER_KI = 0.0;
  public static final double SPINDEXER_KD = 0.0;
  public static final double SPINDEXER_KV = 0.16;
  public static final double SPINDEXER_KA = 0.0;
  public static final double SPINDEXER_KS = 3.9;

  public static final double SPINDEXER_MOI = 0.05;

  public static final double SPINDEXER_VELOCITY_TOLERANCE_RPS = 0.4;

  public static final double KICKER_KP = 11.0;
  public static final double KICKER_KI = 0.0;
  public static final double KICKER_KD = 0.0;
  public static final double KICKER_KV = 0.22;
  public static final double KICKER_KA = 0.0;
  public static final double KICKER_KS = 6.25;

  public static final double KICKER_MOI = 0.01;

  public static final double KICKER_VELOCITY_TOLERANCE_RPS = 3.0;

  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_AMPS =
      SPINDEXER_MOTOR_PEAK_CURRENT_LIMIT - 1.0;
  public static final double SPINDEXER_CURRENT_SPIKE_THRESHOLD_SECONDS = 1.0;
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_AMPS =
      KICKER_MOTOR_PEAK_CURRENT_LIMIT - 1.0;
  public static final double KICKER_CURRENT_SPIKE_THRESHOLD_SECONDS = 1.0;

  public static final double SPINDEXER_UNJAM_VELOCITY_RPS =
      -12; // <-- Should be negative (rotations per second) // FIXME: Update value
  public static final double KICKER_UNJAM_VELOCITY_RPS =
      -60.0; // <-- Should be negative (rotations per second) // FIXME: Update value

  public static final double KICKER_VELOCITY_SETPOINT_1_RPS = 30.0;
  public static final double KICKER_VELOCITY_SETPOINT_2_RPS = 40.0;
  public static final double KICKER_VELOCITY_SETPOINT_3_RPS = 60.0;

  public static final double SPINDEXER_UNJAM_WAIT_TIME = 1.0;
  public static final double KICKER_UNJAM_WAIT_TIME = 0.2;

  public static final double KICKER_FUEL_INTO_SHOOTER_VELOCITY_RPS = 20.0;
  public static final double SPIN_FUEL_INTO_KICKER_VELOCITY_RPS = 12.0;
}
