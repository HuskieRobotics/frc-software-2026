package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class ClimberConstants {

  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_MOTOR_CAN_ID = 17;

  public static final boolean CLIMBER_MOTOR_INVERTED = false;

  public static final double CLIMBER_GEAR_RATIO = 144;

  // PID constants
  public static final double CLIMBER_KP = 10;
  public static final double CLIMBER_KI = 0.0;
  public static final double CLIMBER_KD = 0.0;

  // feed forward constants
  public static final double CLIMBER_KS = 0.0;
  public static final double CLIMBER_KV = 0.0;
  public static final double CLIMBER_KA = 0.0;
  public static final double CLIMBER_KG = 0.0;

  // Motion magic constants
  public static final double CLIMBER_KV_EXPO = 12;
  public static final double CLIMBER_KA_EXPO = 9;
  public static final double CLIMBER_CRUISE_VELOCITY = 50;

  // the following are determined based on the mechanical design of the climber
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.1;

  public static final Voltage RETRACT_VOLTAGE_SLOW = Volts.of(0.0);

  public static final Angle MIN_ANGLE_DEGREES = Degrees.of(0.0);

  public static final Angle MAX_ANGLE_DEGREES = Degrees.of(90);

  public static final Angle ANGLE_TOLERANCE_DEGREES = Degrees.of(1.5);

  public static final double CLIMBER_LENGTH_INCHES =
      7; // FIXME: determine this value based on the mechanical design of the climber

  public static final double CLIMBER_MASS_KG = 2.0;
}
