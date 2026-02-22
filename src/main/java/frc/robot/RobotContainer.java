// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.RobotConfig.CameraConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrainIOXRP;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIO;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIOCTRE;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIONorthstar;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutonomousCommandsFactory;
import frc.robot.commands.CrossSubsystemsCommandsFactory;
import frc.robot.commands.DifferentialDrivetrainCommandFactory;
import frc.robot.commands.IntakeCommandsFactory;
import frc.robot.commands.ShooterCommandsFactory;
import frc.robot.commands.SwerveDrivetrainCommandFactory;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.New2026RobotConfig;
import frc.robot.configs.NewPracticeRobotConfig;
import frc.robot.configs.NorthstarTestPlatformConfig;
import frc.robot.configs.PracticeBoardConfig;
import frc.robot.configs.VisionTestPlatformConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterModes;
import frc.robot.visualizations.RobotVisualization;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private SwerveDrivetrain swerveDrivetrain;
  private DifferentialDrivetrain differentialDrivetrain;
  private Alliance lastAlliance = Field2d.getInstance().getAlliance();
  private Vision vision;
  private Shooter shooter;
  private Hopper hopper;
  private Intake intake;
  private RobotVisualization visualization;

  private ShooterModes shooterModes;

  private Trigger rotateNearBumpTrigger;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();

    constructField();

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT, ROBOT_COMPETITION:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_PRACTICE:
          {
            createCTREPracticeBotSubsystems();
            break;
          }
        case ROBOT_SIMBOT:
          {
            createCTRESimSubsystems();
            break;
          }
        case ROBOT_PRACTICE_BOARD:
          {
            createPracticeBoardSubsystems();
            break;
          }
        case ROBOT_VISION_TEST_PLATFORM:
          {
            createVisionTestPlatformSubsystems();
            break;
          }
        case ROBOT_NORTHSTAR_TEST_PLATFORM:
          {
            createNorthstarTestPlatformSubsystems();
            break;
          }
        case ROBOT_XRP:
          {
            createXRPSubsystems();
            break;
          }
        default:
          break;
      }

    } else {
      swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

      CameraConfig[] cameraConfigs = config.getCameraConfigs();
      VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIO() {};
      }
      vision = new Vision(visionIOs);

      // FIXME: initialize other subsystems
      intake = new Intake(new IntakeIO() {});
      hopper = new Hopper(new HopperIO() {});
      shooter = new Shooter(new ShooterIO() {});
      visualization = new RobotVisualization(intake);
    }

    shooterModes = new ShooterModes(swerveDrivetrain, shooter);

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    // register autonomous commands
    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      AutonomousCommandsFactory.getInstance().configureAutoCommands(differentialDrivetrain, vision);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      AutonomousCommandsFactory.getInstance().configureAutoCommands(swerveDrivetrain, vision);
    }

    // Alert when tuning
    if (Constants.TUNING_MODE) {
      this.tuningAlert.set(true);
    }
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_DEFAULT:
        config = new DefaultRobotConfig();
        break;
      case ROBOT_PRACTICE:
        config = new NewPracticeRobotConfig();
        break;
      case ROBOT_COMPETITION, ROBOT_SIMBOT:
        config = new New2026RobotConfig();
        break;
      case ROBOT_PRACTICE_BOARD:
        config = new PracticeBoardConfig();
        break;
      case ROBOT_VISION_TEST_PLATFORM:
        config = new VisionTestPlatformConfig();
        break;
      case ROBOT_NORTHSTAR_TEST_PLATFORM:
        config = new NorthstarTestPlatformConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIOPhotonVision(
              cameraConfigs[i].id(), FieldConstants.defaultAprilTagType.getLayout());
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIOTalonFX());
    hopper = new Hopper(new HopperIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX());
    visualization = new RobotVisualization(intake);
  }

  private void createCTREPracticeBotSubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIONorthstar(FieldConstants.defaultAprilTagType.getLayout(), cameraConfigs[i]);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIO() {});
    hopper = new Hopper(new HopperIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(intake);
  }

  private void createCTRESimSubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIOSim(
              cameraConfigs[i].id(),
              FieldConstants.defaultAprilTagType.getLayout(),
              swerveDrivetrain::getPose,
              cameraConfigs[i].robotToCameraTransform());
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIOTalonFX());
    hopper = new Hopper(new HopperIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX());
    visualization = new RobotVisualization(intake);
  }

  private void createXRPSubsystems() {
    differentialDrivetrain = new DifferentialDrivetrain(new DifferentialDrivetrainIOXRP());
    vision = new Vision(new VisionIO[] {});
    hopper = new Hopper(new HopperIO() {});
    intake = new Intake(new IntakeIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(intake);
  }

  private void createPracticeBoardSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});
    vision = new Vision(new VisionIO[] {});

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIO() {});
    hopper = new Hopper(new HopperIO() {});
    shooter = new Shooter(new ShooterIOTalonFX());
    visualization = new RobotVisualization(intake);
  }

  private void createVisionTestPlatformSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIOPhotonVision(
              cameraConfigs[i].id(), FieldConstants.defaultAprilTagType.getLayout());
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIO() {});
    hopper = new Hopper(new HopperIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(intake);
  }

  private void createNorthstarTestPlatformSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIONorthstar(FieldConstants.defaultAprilTagType.getLayout(), cameraConfigs[i]);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    intake = new Intake(new IntakeIO() {});
    hopper = new Hopper(new HopperIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(intake);
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});

    Field2d.getInstance().populateAllianceZone();
    Field2d.getInstance().logAllianceZonePoints();
    Field2d.getInstance().populateTrenchZone();
    Field2d.getInstance().logTrenchZonePoints();
    Field2d.getInstance().populateBumpZone();
    Field2d.getInstance().logBumpZonePoints();
    Field2d.getInstance().populateBanks();
    Field2d.getInstance().populatePassingZones();
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

    configureButtonBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();
    configureVisionCommands();

    shooterModes.configureShooterModeTriggers();

    // register commands for other subsystems
    IntakeCommandsFactory.registerCommands(oi, intake);

    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      CrossSubsystemsCommandsFactory.registerCommands(oi, differentialDrivetrain, vision);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      CrossSubsystemsCommandsFactory.registerCommands(
          oi, swerveDrivetrain, intake, hopper, shooter, shooterModes, vision);
      ShooterCommandsFactory.registerCommands(oi, shooter);

      configureRobotContainerTriggers();
    }

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                .withTimeout(1));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5),
                Commands.waitSeconds(0.25),
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5)));
  }

  private void configureDrivetrainCommands() {
    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      DifferentialDrivetrainCommandFactory.registerCommands(oi, differentialDrivetrain);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      SwerveDrivetrainCommandFactory.registerCommands(oi, swerveDrivetrain, vision);
    }
  }

  private void configureVisionCommands() {
    // enable/disable vision
    oi.getVisionIsEnabledTrigger()
        .onTrue(
            Commands.runOnce(() -> vision.enable(true))
                .ignoringDisable(true)
                .withName("enable vision"));
    oi.getVisionIsEnabledTrigger()
        .onFalse(
            Commands.runOnce(() -> vision.enable(false))
                .ignoringDisable(true)
                .withName("disable vision"));
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    // add robot-wide periodic code here
    visualization.update();
  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    this.checkAllianceColor();
  }

  public void configureRobotContainerTriggers() {

    rotateNearBumpTrigger = new Trigger(() -> Field2d.getInstance().inBumpZone());
    rotateNearBumpTrigger.whileTrue(
        CrossSubsystemsCommandsFactory.getRotateWhileNearBumpCommand(swerveDrivetrain));
  }
}
