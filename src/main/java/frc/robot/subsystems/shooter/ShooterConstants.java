package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  // change the ids once we get the updated stuff
  public static final int FLYWHEEL_LEAD_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW1_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW2_MOTOR_ID = 0;
  public static final int FLYWHEEL_FOLLOW3_MOTOR_ID = 0;
  public static final int HOOD_MOTOR_ID = 0;
  public static final int TURRET_MOTOR_ID = 0;

  // PID constants are determined empirically through tuning
  public static final double TOP_SHOOT_KP = 5.0;
  public static final double TOP_SHOOT_KI = 0.0;
  public static final double TOP_SHOOT_KD = 0.0;
  public static final double BOTTOM_SHOOT_KP = 5.0;
  public static final double BOTTOM_SHOOT_KI = 0.0;
  public static final double BOTTOM_SHOOT_KD = 0.0;

  //THIS IS REAL STUFF
  //Flywheel PID Constants (subject to change)
  public static final double FLYWHEEL_LEAD_ROTATION_KP = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KI = 0.0;
  public static final double FLYWHEEL_LEAD_ROTATION_KD = 0.0;

  //Turret PID Constants (subject to change)
  public static final double TURRET_ROTATION_KP = 0.0;
  public static final double TURRET_ROTATION_KI = 0.0;
  public static final double TURRET_ROTATION_KD = 0.0;

  //Hood PID Constants (subject to change)
  public static final double HOOD_ROTATION_KP = 0.0;
  public static final double HOOD_ROTATION_KI = 0.0;
  public static final double HOOD_ROTATION_KD = 0.0;

  //Hard coded non-tunable physics values
  public static final double MOTOR_KS = 0.0;
  public static final double MOTOR_KG = 0.0;
  public static final double ROTATION_KV = 0.0;
  public static final double ROTATION_KA = 0.0;

  //Motion magic PID constants
  public static final double ROTATION_EXPO_KV = 0.0;
  public static final double ROTATION_EXPO_KA = 0.0;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0;
}
