package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_CAN_ID = 17;
  public static final int ANGLE_ENCODER_ID = 18; // FIXME: set to actual CAN ID

  public static final boolean CLIMBER_MOTOR_INVERTED = false; // FIXME: verify direction

  public static final double CLIMBER_GEAR_RATIO = 144.0;

  public static final double ENCODER_GEAR_RATIO = 1.0;

  public static final double CLIMBER_MAGNET_OFFSET = 0.0;

  public static final double CLIMBER_KP = 12.0; // FIXME: tune
  public static final double CLIMBER_KI = 0.0;
  public static final double CLIMBER_KD = 0.0;

  // Feedforward constants (determined through SysId)
  public static final double CLIMBER_KS = 0.13; // FIXME: run SysId
  public static final double CLIMBER_KV = 0.3; // FIXME: run SysId
  public static final double CLIMBER_KA = 0.123; // FIXME: run SysId
  public static final double CLIMBER_KG = 1.5; // FIXME: run SysId

  public static final double CLIMBER_KV_EXPO = 0.5; // FIXME: tune
  public static final double CLIMBER_KA_EXPO = 0.15; // FIXME: tune
  public static final double CLIMBER_CRUISE_VELOCITY = 0; // don't limit cruise velocity

  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 80;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.5;

  public static final Angle MIN_ANGLE_DEGREES = Degrees.of(-72.6);
  public static final Angle MAX_ANGLE_DEGREES = Degrees.of(144.2);
  public static final Angle ANGLE_TOLERANCE_DEGREES = Degrees.of(0.5);

  public static final double CLIMBER_LENGTH_INCHES = 7.0;
  public static final double CLIMBER_MASS_KG = 0.69444992;
}
