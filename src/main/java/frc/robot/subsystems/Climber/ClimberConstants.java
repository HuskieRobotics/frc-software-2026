package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class ClimberConstants {

  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_MOTOR_CAN_ID = 17;
  public static final int ANGLE_ENCODER_ID =
      0; // FIXME: determine this value based on the CAN ID assigned to the angle encoder

  public static final boolean CLIMBER_MOTOR_INVERTED = false;

  public static final double CLIMBER_GEAR_RATIO = 144;
  public static final double ENCODER_GEAR_RATIO = 5; // FIXME: update this value

  // PID constants
  public static final double CLIMBER_KP =
      12; // FIXME: determine this value based on tuning of the climber
  public static final double CLIMBER_KI =
      0.0; // FIXME: determine this value based on tuning of the climber
  public static final double CLIMBER_KD =
      0.0; // FIXME: determine this value based on tuning of the climber

  // feed forward constants
  public static final double CLIMBER_KS =
      0.13; // FIXME: determine this value based on testing of the climber
  public static final double CLIMBER_KV =
      0.3; // FIXME: determine this value based on testing of the climber
  public static final double CLIMBER_KA =
      0.123; // FIXME: determine this value based on testing of the climber
  public static final double CLIMBER_KG =
      1.5; // FIXME: determine this value based on testing of the climber

  // Motion magic constants
  public static final double CLIMBER_KV_EXPO =
      0.5; // FIXME: determine this value based on testing of the climber
  public static final double CLIMBER_KA_EXPO =
      0.15; // FIXME: determine this value based on testing of the climber
  public static final double CLIMBER_CRUISE_VELOCITY =
      0; // FIXME: determine this value based on testing of the climber

  // the following are determined based on the mechanical design of the climber
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 80;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.5;

  public static final Angle MIN_ANGLE_DEGREES =
      Degrees.of(
          -72.6); // FIXME: determine this value based on the mechanical design of the climber

  public static final Angle MAX_ANGLE_DEGREES =
      Degrees.of(90); // FIXME: determine this value based on the mechanical design of the climber

  public static final Angle ANGLE_TOLERANCE_DEGREES = Degrees.of(.5);

  public static final double CLIMBER_LENGTH_INCHES =
      7; // FIXME: determine this value based on the mechanical design of the climber

  public static final double CLIMBER_MASS_KG = 0.69444992;
}
