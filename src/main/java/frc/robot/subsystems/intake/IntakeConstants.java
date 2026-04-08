package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {

  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int ROLLER_MOTOR_ID = 42;
  public static final int DEPLOYER_MOTOR_ID = 33;

  public static final double DEPLOYER_KP = 23.0;
  public static final double DEPLOYER_KI = 0;
  public static final double DEPLOYER_KD = 0;
  public static final double DEPLOYER_KS = 0.49;
  public static final double DEPLOYER_KV = 0.0;
  public static final double DEPLOYER_KA = 0.0;
  public static final double DEPLOYER_KG = 0.0;

  public static final double DEPLOYER_GEAR_RATIO = 11.25;
  public static final boolean DEPLOYER_MOTOR_INVERTED = true;

  // Mechanical properties for simulation
  public static final double DEPLOYER_LENGTH_METERS = 0.3;
  public static final double DEPLOYER_MASS_KG = 0.15;
  public static final double DEPLOYER_MIN_ANGLE_ROT = 0.0;
  public static final double DEPLOYER_MAX_ANGLE_ROT = 23.3;

  public static final double DEPLOYER_CIRCUMFERENCE_METERS = Units.inchesToMeters(Math.PI);

  public static final double DEPLOYED_ANGULAR_POSITION_ROT = 3.6606; // 11.5 inches
  public static final double DEPLOYED_LINEAR_POSITION_METERS =
      DEPLOYED_ANGULAR_POSITION_ROT * DEPLOYER_CIRCUMFERENCE_METERS;

  public static final double RETRACTED_ANGULAR_POSITION_ROT = 0.0;
  public static final double RETRACTED_LINEAR_POSITION_METERS = Units.inchesToMeters(0.1);

  public static final double DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS = Units.inchesToMeters(0.5);
  public static final double DEPLOYER_ANGULAR_POSITION_TOLERANCE_ROT =
      DEPLOYER_LINEAR_POSITION_TOLERANCE_METERS / DEPLOYER_CIRCUMFERENCE_METERS;
  public static final double DEPLOYER_HOPPER_INTERFERENCE_LIMIT_METERS = Units.inchesToMeters(3.0);
  public static final double INTAKE_SLOW_JOSTLE_DEPLOYER_CURRENT = -10;

  public static final double DEPLOYER_SUPPLY_CURRENT_LIMIT = 20.0;
  public static final double DEPLOYER_STATOR_CURRENT_LIMIT = 20.0;

  public static final double ROLLER_KP = 0.4; // 2.675
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0.4; // 5.8
  public static final double ROLLER_KV = 0.195;
  public static final double ROLLER_KA = 0;

  // this is the gear ratio for the wheels; the roller, which is mechanically linked to the wheel
  // shaft, has a different gear ratio, but we will control the velocity of the wheels
  public static final double ROLLER_GEAR_RATIO = 2;
  public static final boolean ROLLER_MOTOR_INVERTED = true;

  public static final double ROLLER_TARGET_VELOCITY_RPS = 40.0;
  public static final double ROLLER_AUTO_TARGET_VELOCITY_RPS = 50.0;

  public static final double ROLLER_EJECT_VELOCITY_RPS = -40.0;
  public static final double ROLLER_VELOCITY_TOLERANCE_RPS = 3.0;

  public static final double ROLLER_PEAK_CURRENT_LIMIT = 80.0;
  public static final double ROLLER_CONTINUOUS_CURRENT_LIMIT = 60.0;
  public static final double ROLLER_PEAK_CURRENT_DURATION = 0.1;

  public static final double DEPLOYER_HOLD_POSITION_CURRENT_LIMIT = 10;

  public static final double ROLLER_JAMMED_CURRENT_AMPS = 39.0;
  public static final double ROLLER_JAMMED_TIME_THRESHOLD_SECONDS = 0.2;
  public static final double ROLLER_UNJAM_DURATION_SECONDS = 2.0;

  public static final double ROLLER_STALL_VELOCITY_THRESHOLD_RPS = 5.0;

  public static final double DEPLOYER_JOSTLE_FUEL_CURRENT = -20.0;

  public static final double JOSTLE_FIRST_RETRACT_POSITION_METERS = Units.inchesToMeters(5.0);
  public static final double JOSTLE_SUBSEQUENT_RETRACT_POSITION_METERS =
      Units.inchesToMeters(3.5); // possibly change to 4

  public static final double JOSTLE_EXTENDED_POSITION_METERS = Units.inchesToMeters(11.0);
  public static final int JOSTLE_INITIAL_FUEL_COUNT = 10;
}
