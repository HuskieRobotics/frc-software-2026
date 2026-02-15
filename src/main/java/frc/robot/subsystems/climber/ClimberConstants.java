package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class ClimberConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_CAN_ID = 17;
  public static final int ANGLE_ENCODER_ID = 18; // FIXME: set to actual CAN ID

  public static final boolean CLIMBER_MOTOR_INVERTED = true; // FIXME: verify direction
  public static final boolean CLIMBER_ENCODER_INVERTED = false; // FIXME: verify direction

  public static final double CLIMBER_GEAR_RATIO = 144.0;

  public static final double ENCODER_GEAR_RATIO = 1.0;

  public static final double CLIMBER_MAGNET_OFFSET = 0.0;

  public static final double CLIMBER_KP = 120.0; // FIXME: tune
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

  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT =
      100; // keep this higher as we will be supporting the weight of the robot
  public static final double CLIMBER_PEAK_CURRENT_LIMIT =
      120; // keep this higher as we will be supporting the weight of the robot
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.5;

  // 0 degrees is the position where the hooks are horizontal inside of the robot; the angle
  // increases as the hooks deploy to climb
  public static final Angle MIN_ANGLE_DEGREES =
      Degrees.of(-80.0); // we will call our starting angle 0 degrees
  public static final Angle MAX_ANGLE_DEGREES = Degrees.of(160.0); // FIXME: set to actual max angle
  public static final Angle ANGLE_TOLERANCE_DEGREES = Degrees.of(5.0);

  public static final double CLIMBER_LENGTH_INCHES = 7.0;
  public static final double CLIMBER_MASS_KG = 0.69444992;

  public static final Angle CLIMB_READY_ANGLE = Degrees.of(160); // FIXME: hook ready to grab rung
  public static final Angle CLIMB_RETRACT_ANGLE = Degrees.of(90.0); // FIXME: pull robot up

  public static final Voltage CLIMBER_EXTEND_VOLTAGE =
      Volts.of(6.0); // FIXME: find correct value for extending the climber
  public static final Voltage CLIMBER_RETRACT_VOLTAGE =
      Volts.of(-6.0); // FIXME: find correct value for retracting the climber
  public static final Voltage CLIMBER_CLIMB_VOLTAGE = Volts.of(-12.0);
}
