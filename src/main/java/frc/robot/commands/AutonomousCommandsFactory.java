package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.subsystems.hopper.Hopper;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandsFactory {

  private static AutonomousCommandsFactory autonomousCommandFactory = null;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static AutonomousCommandsFactory getInstance() {
    if (autonomousCommandFactory == null) {
      autonomousCommandFactory = new AutonomousCommandsFactory();
    }
    return autonomousCommandFactory;
  }

  private AutonomousCommandsFactory() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(SwerveDrivetrain drivetrain, Vision vision, Hopper hopper) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    autoChooser.addOption("Right to Neutral Zone x2", rightToNeutralZoneX2(hopper));

    autoChooser.addOption("Left to Neutral Zone x2", leftToNeutralZoneX2(hopper));

    autoChooser.addOption("Left Neutral Zone And Depot", leftNeutralZoneAndDepot(hopper));

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    autoChooser.addOption(
        "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    autoChooser.addOption(
        "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    autoChooser.addOption(
        "Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption("Drive Velocity Tuning", this.getDriveVelocityTuningCommand(drivetrain));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    autoChooser.addOption( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Characterization",
        this.getDriveWheelRadiusCharacterizationCommand(drivetrain));

    // autoChooser.addOption(
    //     "Left Neutral Zone Hopper and Climb",
    //     leftNeutralZoneHopperAndClimb(drivetrain)); // add shooter later

    // autoChooser.addOption(
    //     "Right Neutral Zone Hopper and Climb",
    //     rightNeutralZoneHopperAndClimb(drivetrain)); // add shooter later

    // autoChooser.addOption("Middle Depot Hopper and Climb",
    // middleDepotHopperAndClimb(drivetrain));

    // autoChooser.addOption("Left Depot Hopper and Climb", leftDepotHopperAndClimb(drivetrain));
    // autoChooser.addOption("Middle Hopper and Climb", middleHopperAndClimb(drivetrain));

  }

  public void configureAutoCommands(DifferentialDrivetrain drivetrain, Vision vision) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Differential Auto ************
     *
     * example PathPlanner auto for XRP
     *
     */

    autoChooser.addOption("Differential Example", new PathPlannerAuto("Differential Auto"));
  }

  private Command getDriveVelocityTuningCommand(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(2.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-0.5),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(1.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(3.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(1.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-1.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-3.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-1.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(0.0),
                            false,
                            false),
                    drivetrain))));
  }

  private Command getSwerveRotationTuningCommand(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(0.1),
                            MetersPerSecond.of(0.1),
                            RadiansPerSecond.of(0.0),
                            true,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-0.1),
                            MetersPerSecond.of(0.1),
                            RadiansPerSecond.of(0.0),
                            true,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(-0.1),
                            MetersPerSecond.of(-0.1),
                            RadiansPerSecond.of(0.0),
                            true,
                            false),
                    drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(
                    () ->
                        drivetrain.drive(
                            MetersPerSecond.of(0.1),
                            MetersPerSecond.of(-0.1),
                            RadiansPerSecond.of(0.0),
                            true,
                            false),
                    drivetrain))));
  }

  private Command getDriveWheelRadiusCharacterizationCommand(SwerveDrivetrain drivetrain) {
    return CharacterizationCommands.wheelRadiusCharacterization(drivetrain);
  }

  private Command createTuningAutoPath(
      String autoName, boolean measureDistance, SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  }

  private Command getUnloadHopperFromBankCommand(Hopper hopper) {
    return Commands.parallel(
        Commands.run(
            () ->
                hopper.setKickerVelocity(
                    RotationsPerSecond.of(35.0))), // FIXME: get shooter flywheel velocity
        Commands.run(() -> hopper.setSpindexerVelocity(RotationsPerSecond.of(4.5))));
  }

  private Command getUnloadHopperFromDepotCommand(Hopper hopper) {
    return Commands.parallel(
        Commands.run(
            () ->
                hopper.setKickerVelocity(
                    RotationsPerSecond.of(38.0))), // FIXME: get shooter flywheel velocity
        Commands.run(() -> hopper.setSpindexerVelocity(RotationsPerSecond.of(4.5))));
  }

  private Command leftNeutralZoneHopperAndClimb(
      SwerveDrivetrain drivetrain /*, Shooter shooter*/) { // add shooter and intake later
    PathPlannerPath driveToNeutralZone;
    PathPlannerPath driveToBank;
    PathPlannerPath driveToTower;
    try {
      driveToNeutralZone = PathPlannerPath.fromPathFile("Left to Far NZ");
      driveToBank = PathPlannerPath.fromPathFile("Left Far NZ to Bank");
      driveToTower = PathPlannerPath.fromPathFile("Left Bank to Tower");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }
    // 1st priority for bluff
    // follow path to neutral zone
    // change pathplanner path to collect fuel
    // follow path to near bank
    // drive to bank command
    // unload shoot for x seconds
    // follow path to tower
    // climb sequence command

    return Commands.sequence(
        AutoBuilder.followPath(driveToNeutralZone),
        Commands.waitSeconds(1),
        AutoBuilder.followPath(driveToBank),
        Commands.waitSeconds(4),
        AutoBuilder.followPath(driveToTower));
  }

  private Command rightNeutralZoneHopperAndClimb(
      SwerveDrivetrain drivetrain /*, Shooter shooter*/) { // add shooter and intake later
    PathPlannerPath driveToNeutralZone;
    PathPlannerPath driveToBank;
    PathPlannerPath driveToTower;
    try {
      driveToNeutralZone = PathPlannerPath.fromPathFile("Right to Far NZ");
      driveToBank = PathPlannerPath.fromPathFile("Right Far NZ to Bank");
      driveToTower = PathPlannerPath.fromPathFile("Right Bank to Tower");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    // 1st priority for bluffs
    // follow path to neutral zone
    // collect fuel with timeout x OR another PathPlannerPath to drive along the NZ
    // follow path to near bank
    // drive to bank command
    // unload shooter for x seconds
    // follow path to tower
    // climb sequence command
    return Commands.sequence(
        AutoBuilder.followPath(driveToNeutralZone),
        Commands.waitSeconds(1),
        AutoBuilder.followPath(driveToBank),
        Commands.waitSeconds(1),
        AutoBuilder.followPath(driveToTower),
        new DriveToPose(
            drivetrain,
            this::getTowerPose,
            CrossSubsystemsCommandsFactory.xController,
            CrossSubsystemsCommandsFactory.yController,
            CrossSubsystemsCommandsFactory.thetaController,
            new Transform2d(
                Units.inchesToMeters(1.0), Units.inchesToMeters(1.0), Rotation2d.fromDegrees(5)),
            true,
            (atPose) ->
                LEDs.getInstance()
                    .requestState(atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
            CrossSubsystemsCommandsFactory::updatePIDConstants,
            5));

    // return Commands.sequence(
    //     AutoBuilder.followPath(driveToNeutralZone),
    //     Commands.waitSeconds(1),
    //     AutoBuilder.followPath(driveToBank),
    //     Commands.waitSeconds(1),
    //     AutoBuilder.followPath(driveToTower));
  }

  private Command middleDepotHopperAndClimb(
      SwerveDrivetrain drivetrain /*, Shooter shooter*/) { // add shooter and intake later
    PathPlannerPath driveToDepot;
    PathPlannerPath depotToTower;
    try {
      driveToDepot = PathPlannerPath.fromPathFile("Middle Go To Depot");
      depotToTower = PathPlannerPath.fromPathFile("Depot to Tower");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }
    // 2nd Prority for Bluffs
    // follow path to depot
    // collect fuel
    // unload shooter
    // follow path to tower
    // climb sequence

    return Commands.sequence(
        AutoBuilder.followPath(driveToDepot),
        Commands.waitSeconds(1),
        AutoBuilder.followPath(depotToTower));
  }

  private Pose2d getTowerPose() {
    double xOffset =
        RobotConfig.getInstance().getRobotWidthWithBumpers().in(Meters) / 2.0
            + Units.inchesToMeters(2.0);
    double yOffset = 0.0; // 6 inch buffer from the edge of the robot to the tower

    // temp solution for tower pose
    return new Pose2d(
        FieldConstants.Tower.rightUpright.getX() + xOffset,
        FieldConstants.Tower.rightUpright.getY() + yOffset,
        Rotation2d.fromDegrees(-90));
  }

  private Command leftDepotHopperAndClimb(
      SwerveDrivetrain drivetrain /*, Shooter shooter*/) { // add shooter and intake later
    PathPlannerPath driveToDepot;
    PathPlannerPath depotToTower;
    try {
      driveToDepot = PathPlannerPath.fromPathFile("Left Go to Depot");
      depotToTower = PathPlannerPath.fromPathFile("Depot to Tower");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }
    // 2nd Prority for Bluffs
    // follow path to depot
    // collect fuel
    // unload shooter
    // follow path to tower
    // climb sequence

    return Commands.sequence(
        AutoBuilder.followPath(driveToDepot),
        Commands.waitSeconds(1),
        AutoBuilder.followPath(depotToTower));
  }

  private Command middleHopperAndClimb(
      SwerveDrivetrain drivetrain /*, Shooter shooter*/) { // add shooter and intake later
    PathPlannerPath driveToTower;
    try {
      driveToTower = PathPlannerPath.fromPathFile("Middle Shoot Preload+Climb");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      // Low priority for bluffs
      // unload shooter
      // follow path to tower
      // climb sequence
      return Commands.none();
    }

    return Commands.sequence(AutoBuilder.followPath(driveToTower));
  }

  private Command rightToNeutralZoneX2(Hopper hopper) { // add shooter and intake later
    PathPlannerPath driveToNeutralZoneAndBack;
    PathPlannerPath driveToNeutralZoneAgain;
    PathPlannerPath driveToBank;
    try {
      driveToNeutralZoneAndBack =
          PathPlannerPath.fromPathFile("R Fuel Sweep"); // this path ends at the bank
      driveToNeutralZoneAgain = PathPlannerPath.fromPathFile("R Sweep Collect");
      driveToBank = PathPlannerPath.fromPathFile("R Sweep to Bank");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      // 3rd priority for Bluffs
      // Follow path to neutral zone
      // collect fuel
      // follow path to bank
      // unload shoot
      // repeat
      return Commands.none();
    }
    return Commands.sequence(
        AutoBuilder.followPath(driveToNeutralZoneAndBack),
        getUnloadHopperFromBankCommand(hopper).withTimeout(5.0),
        Commands.runOnce(hopper::stop),
        AutoBuilder.followPath(driveToNeutralZoneAgain),
        AutoBuilder.followPath(driveToBank),
        getUnloadHopperFromBankCommand(hopper));
  }

  private Command leftToNeutralZoneX2(Hopper hopper) { // add shooter and intake later
    PathPlannerPath driveToNeutralZoneAndBack;
    PathPlannerPath driveToNeutralZoneAgain;
    PathPlannerPath driveToBank;
    try {
      driveToNeutralZoneAndBack =
          PathPlannerPath.fromPathFile("L Fuel Sweep"); // this path ends at the bank
      driveToNeutralZoneAgain = PathPlannerPath.fromPathFile("L Sweep Collect");
      driveToBank = PathPlannerPath.fromPathFile("L Sweep to Bank");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      // 3rd priority for bluffs
      // follow path to neutral zone
      // collect fuel
      // follow path to bank
      // unload shoot
      // repeat
      return Commands.none();
    }

    return Commands.sequence(
        AutoBuilder.followPath(driveToNeutralZoneAndBack),
        getUnloadHopperFromBankCommand(hopper).withTimeout(5.0),
        Commands.runOnce(hopper::stop),
        AutoBuilder.followPath(driveToNeutralZoneAgain),
        AutoBuilder.followPath(driveToBank),
        getUnloadHopperFromBankCommand(hopper));
  }

  private Command leftNeutralZoneAndDepot(Hopper hopper) {
    PathPlannerPath driveToNeutralZoneAndBack;
    PathPlannerPath driveToDepot;
    PathPlannerPath intakeFromDepot;
    try {
      driveToNeutralZoneAndBack = PathPlannerPath.fromPathFile("L Fuel Sweep");
      driveToDepot = PathPlannerPath.fromPathFile("Left Bank to Depot");
      intakeFromDepot = PathPlannerPath.fromPathFile("Intake Depot");
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    // consider not shooting at the bank and going straight to the depot
    return Commands.sequence(
        AutoBuilder.followPath(driveToNeutralZoneAndBack),
        getUnloadHopperFromBankCommand(hopper).withTimeout(5.0),
        Commands.runOnce(hopper::stop),
        AutoBuilder.followPath(driveToDepot),
        Commands.parallel(
            getUnloadHopperFromDepotCommand(hopper), AutoBuilder.followPath(intakeFromDepot)));
  }
}
