package frc.robot.subsystems.fuelIntake;

import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private IntakeConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

    public static final double DEPLOYER_KP = 0;
    public static final double DEPLOYER_KI = 0;
    public static final double DEPLOYER_KD = 0;
    public static final double DEPLOYER_KS = 0;
    public static final double DEPLOYER_KV = 0;
    public static final double DEPLOYER_KA = 0;
    public static final double DEPLOYER_KG = 0;
    public static final double DEPLOYER_KV_EXPO = 0;
    public static final double DEPLOYER_KA_EXPO = 0;


    public static final int ROLLER_MOTOR_ID = 0;
    public static final int DEPLOYER_MOTOR_ID = 0;


    public static final Distance DEPLOYER_CIRCUIMFERENCE = null;
    public static final double DEPLOYER_CRUISE_VELOCITY = 0;
    public static final double DEPLOYER_PEAK_CURRENT_LIMIT = 0;
    public static final double DEPLOYER_LOWER_CURRENT_LIMIT = 0;


    public static final String SUBSYSTEM_NAME = "Intake";


    public static final double ROLLER_PEAK_CURRENT_LIMIT = 0;
    public static final double ROLLER_LOWER_CURRENT_LIMIT = 0;
    public static final double INTAKE_ROLLER_VELOCITY_RPS = 0;
    public static final Distance DEPLOYED_POSITION_ROTATIONS = null;
    public static final Distance RETRACTED_POSITION_ROTATIONS = null;

   
  }


