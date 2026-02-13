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

  public static final double DEPLOYER_GEAR_RATIO = 50.0; // FIXME: set value
  public static final boolean DEPLOYER_MOTOR_INVERTED = false; // FIXME: check direction

  // Mechanical properties for simulation
  public static final double DEPLOYER_MOI = 0.25;
  public static final double DEPLOYER_LENGTH_METERS = 0.3;
  public static final double DEPLOYER_MASS_KG = 3.0;
  public static final Angle DEPLOYER_MIN_ANGLE = Degrees.of(0);
  public static final Angle DEPLOYER_MAX_ANGLE = Degrees.of(90);

  public static final Angle DEPLOYED_POSITION = Rotations.of(0.25); // FIXME: set value
  public static final Angle RETRACTED_POSITION = Rotations.of(0); // FIXME: set value
  public static final Distance DEPLOYER_CIRCUMFERENCE = Inches.of(1.2);
  public static final Distance DEPLOYER_LINEAR_POSITION_TOLERANCE = Inches.of(1.5);
  public static final Angle DEPLOYER_POSITION_TOLERANCE = Rotations.of(0.05);

  public static final AngularVelocity DEPLOYER_CRUISE_VELOCITY = RotationsPerSecond.of(1.0);
  public static final Voltage DEPLOYER_RETRACT_VOLTAGE = Volts.of(-3);

  public static final double DEPLOYER_PEAK_CURRENT_LIMIT = 80;
  public static final double DEPLOYER_CONTINUOUS_CURRENT_LIMIT = 50;
  public static final double DEPLOYER_PEAK_CURRENT_DURATION = 0.5;

  public static final double ROLLER_KP = 0.1; // FIXME: tune value
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0.25;
  public static final double ROLLER_KV = 0.12;
  public static final double ROLLER_KA = 0.01;

  public static final double ROLLER_GEAR_RATIO = 2.0; // FIXME: set value
  public static final boolean ROLLER_MOTOR_INVERTED = false; // FIXME: check direction

  public static final AngularVelocity ROLLER_TARGET_VELOCITY = RotationsPerSecond.of(50);
  public static final AngularVelocity ROLLER_EJECT_VELOCITY = RotationsPerSecond.of(-20);
  public static final AngularVelocity ROLLER_VELOCITY_TOLERANCE = RotationsPerSecond.of(2.0);
  public static final AngularVelocity ROLLER_CRUISE_VELOCITY = RotationsPerSecond.of(60);

  public static final double ROLLER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double ROLLER_PEAK_CURRENT_LIMIT = 20;
  public static final double DEPLOYER_HOLD_POSITION_CURRENT_LIMIT = 10;

  public static final double ROLLER_JAMMED_CURRENT_AMPS = 50;
  public static final double ROLLER_JAMMED_TIME_THRESHOLD_SECONDS = 0.2;
}
