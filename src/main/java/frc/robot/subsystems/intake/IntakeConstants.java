package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public class IntakeConstants {

  public static final String SUBSYSTEM_NAME = "Intake";

  public static final int ROLLER_MOTOR_ID = 57;
  public static final int DEPLOYER_MOTOR_ID = 55;

  public static final double DEPLOYER_KP = 12; // FIXME: tune value
  public static final double DEPLOYER_KI = 0;
  public static final double DEPLOYER_KD = 0;
  public static final double DEPLOYER_KS = 0.25;
  public static final double DEPLOYER_KV = 0.37;
  public static final double DEPLOYER_KA = 0.583;
  public static final double DEPLOYER_KG = 0.45;

  public static final double DEPLOYER_GEAR_RATIO = 4.0; // FIXME: set value
  public static final boolean DEPLOYER_MOTOR_INVERTED = true; // FIXME: check direction

  // Mechanical properties for simulation
  public static final double DEPLOYER_LENGTH_METERS = 0.3;
  public static final double DEPLOYER_MASS_KG = 1.5;
  public static final Angle DEPLOYER_MIN_ANGLE = Rotations.of(0); // FIXME: set value
  public static final Angle DEPLOYER_MAX_ANGLE = Rotations.of(1.4); // FIXME: set value

  public static final Distance DEPLOYER_CIRCUMFERENCE = Inches.of(1.0).times(Math.PI);
  public static final Distance DEPLOYED_LINEAR_POSITION = Inches.of(10.0); // FIXME: set value
  public static final Angle DEPLOYED_ANGULAR_POSITION =
      Rotations.of(DEPLOYED_LINEAR_POSITION.div(DEPLOYER_CIRCUMFERENCE).magnitude()); // FIXME: set value
  public static final Distance RETRACTED_LINEAR_POSITION = Inches.of(0.0); // FIXME: set value
  public static final Angle RETRACTED_ANGULAR_POSITION =
      Rotations.of(RETRACTED_LINEAR_POSITION.div(DEPLOYER_CIRCUMFERENCE).magnitude()); // FIXME: set value
  public static final Distance DEPLOYER_LINEAR_POSITION_TOLERANCE = Inches.of(0.25);
  public static final Angle DEPLOYER_ANGULAR_POSITION_TOLERANCE = Rotations.of(DEPLOYER_LINEAR_POSITION_TOLERANCE.div(DEPLOYER_CIRCUMFERENCE).magnitude());

  public static final double DEPLOYER_PEAK_CURRENT_LIMIT = 60;
  public static final double DEPLOYER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double DEPLOYER_PEAK_CURRENT_DURATION = 0.5;

  public static final double ROLLER_KP = 12; // FIXME: tune value
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0.25;
  public static final double ROLLER_KV = 0.12;
  public static final double ROLLER_KA = 0.01;

  // this is the gear ratio for the wheels; the roller, which is mechanically linked to the wheel
  // shaft, has a different gear ratio, but we will control the velocity of the wheels
  public static final double ROLLER_GEAR_RATIO = 2.14;
  public static final boolean ROLLER_MOTOR_INVERTED = false; // FIXME: check direction

  public static final AngularVelocity ROLLER_TARGET_VELOCITY =
      RadiansPerSecond.of(50); // FIXME: set value
  public static final AngularVelocity ROLLER_EJECT_VELOCITY = RadiansPerSecond.of(-20);
  public static final AngularVelocity ROLLER_VELOCITY_TOLERANCE = RadiansPerSecond.of(2.0);

  public static final double ROLLER_PEAK_CURRENT_LIMIT = 40;

  public static final double DEPLOYER_HOLD_POSITION_CURRENT_LIMIT = 10;

  public static final double ROLLER_JAMMED_CURRENT_AMPS = 50;
  public static final double ROLLER_JAMMED_TIME_THRESHOLD_SECONDS = 0.2;
  public static final double ROLLER_UNJAM_DURATION_SECONDS = 2.0;
}
