package frc.robot.subsystems.fuelIntake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private IntakeConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // FIXME: update all values

  public static final double DEPLOYER_KP = 0;
  public static final double DEPLOYER_KI = 0;
  public static final double DEPLOYER_KD = 0;
  public static final double DEPLOYER_KS = 0;
  public static final double DEPLOYER_KV = 0;
  public static final double DEPLOYER_KA = 0;
  public static final double DEPLOYER_KG = 0;
  public static final double DEPLOYER_KV_EXPO = 0;
  public static final double DEPLOYER_KA_EXPO = 0;

  public static final double ROLLER_KP = 0;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0;
  public static final double ROLLER_KA = 0;
  public static final double ROLLER_KV_EXPO = 0;
  public static final double ROLLER_KA_EXPO = 0;
  public static final double ROLLER_JERK = 0.0;

  public static final int ROLLER_MOTOR_ID = 57;
  public static final int DEPLOYER_MOTOR_ID = 55;

  public static final Distance DEPLOYER_CIRCUMFERENCE =
      Meters.of(0.03048 * Math.PI); // 1.2 inches to meters
  public static final double DEPLOYER_CRUISE_VELOCITY = 0;
  public static final double DEPLOYER_PEAK_CURRENT_LIMIT = 40;
  public static final double DEPLOYER_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double DEPLOYER_HOLD_POSITION_CURRENT_LIMIT = 10;
  public static final double DEPLOYER_PEAK_CURRENT_DURATION = 0.5;

  public static final String SUBSYSTEM_NAME = "Intake";

  public static final double ROLLER_CRUISE_VELOCITY = 0;
  public static final double ROLLER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final AngularVelocity INTAKE_ROLLER_VELOCITY = RotationsPerSecond.of(0);
  public static final Distance DEPLOYED_POSITION = null; // FIXME: set value
  public static final Distance RETRACTED_POSITION = null; // FIXME: set value

  public static final double ROLLER_JAMMED_CURRENT_AMPS = 50;
  public static final double ROLLER_JAMMED_TIME_THRESHOLD_SECONDS = 0;
  public static final double DEPLOYED_POSITION_TOLERANCE = 0;
  public static final double RETRACTED_POSITION_TOLERANCE = 0;

  public static final double ROLLER_GEAR_RATIO = 0.0; // FIXME: set value
  public static final double DEPLOYER_GEAR_RATIO = 0.0; // FIXME: set value
}
