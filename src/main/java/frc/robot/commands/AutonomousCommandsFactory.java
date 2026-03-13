package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandsFactory {

  private static AutonomousCommandsFactory autonomousCommandFactory = null;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  private Timer hopperUnloadTimer;
  private Timer matchTimer;

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

  private AutonomousCommandsFactory() {
    matchTimer = new Timer();
    hopperUnloadTimer = new Timer();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(
      SwerveDrivetrain drivetrain, Vision vision, Hopper hopper, Intake intake, Shooter shooter) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    autoChooser.addOption(
        "Right Neutral Zone x2", rightToNeutralZoneX2(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Right Neutral Zone and Outpost",
        rightNeutralZoneAndOutpost(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Left Neutral Zone x2", leftToNeutralZoneX2(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Safe Right Neutral Zone x2",
        safeRightToNeutralZoneX2(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Safe Left Neutral Zone x2", safeLeftToNeutralZoneX2(drivetrain, hopper, intake, shooter));

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

  private Command getUnloadHopperCommand(
      Hopper hopper, Intake intake, Shooter shooter, boolean checkForFuel) {

    return Commands.sequence(
        Commands.runOnce(() -> hopperUnloadTimer.restart()),
        Commands.parallel(
            hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocity),
            Commands.repeatingSequence( // FIXME: to change
                    Commands.run(intake::jostleFuelIn, intake).withTimeout(0.4),
                    Commands.run(intake::jostleFuelOut, intake).withTimeout(0.2))
                .until(
                    () ->
                        (checkForFuel
                            && hopperUnloadTimer.get() > 2.0
                            && !shooter.getFuelDetected()))),
        Commands.runOnce(hopper::stop, hopper),
        Commands.runOnce(intake::deployIntake, intake));
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

  private Command getNeutralZoneSweepAuto(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      Pose2d startingPose,
      PathPlannerPath firstSweep,
      PathPlannerPath secondSweepCollect,
      PathPlannerPath secondSweepToBank) {
    return Commands.sequence(
            Commands.runOnce(matchTimer::restart),
            setStartingPoseForAuto(startingPose, drivetrain),
            Commands.parallel(
                intake.getDeployAndStartCommand(), AutoBuilder.followPath(firstSweep)),
            // alternate between unloading hopper and running sweeps until not enough time left in
            // the auto to do so
            Commands.repeatingSequence(
                getUnloadHopperCommand(hopper, intake, shooter, true),
                Commands.runOnce(hopper::stop, hopper),
                Commands.either(
                    Commands.sequence(
                        AutoBuilder.followPath(secondSweepCollect),
                        // check if we have time to come back from the sweep
                        Commands.either(
                            AutoBuilder.followPath(secondSweepToBank),
                            Commands.none(),
                            () -> matchTimer.get() < 12.5)),
                    getUnloadHopperCommand(hopper, intake, shooter, true),
                    (() -> matchTimer.get() < 12.5))))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
            });
  }

  private Command setStartingPoseForAuto(Pose2d startingPose, SwerveDrivetrain drivetrain) {
    return Commands.runOnce(
        () -> {
          Pose2d pose = startingPose;
          if (drivetrain.shouldFlipAutoPath()) {
            pose = FlippingUtil.flipFieldPose(startingPose);
          }
          drivetrain.resetPose(pose);
        });
  }

  private Command rightToNeutralZoneX2(
      SwerveDrivetrain drivetrain, Hopper hopper, Intake intake, Shooter shooter) {
    PathPlannerPath firstFuelSweep;
    PathPlannerPath secondSweepCollect;
    PathPlannerPath secondSweepToBank;
    final Pose2d startingPose;
    try {
      firstFuelSweep = PathPlannerPath.fromPathFile("R Fuel Sweep");
      secondSweepCollect = PathPlannerPath.fromPathFile("R Sweep Collect");
      secondSweepToBank = PathPlannerPath.fromPathFile("R Sweep to Bank");
      startingPose = firstFuelSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getNeutralZoneSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        startingPose,
        firstFuelSweep,
        secondSweepCollect,
        secondSweepToBank);
  }

  private Command safeRightToNeutralZoneX2(
      SwerveDrivetrain drivetrain, Hopper hopper, Intake intake, Shooter shooter) {
    PathPlannerPath safeFirstFuelSweep;
    PathPlannerPath secondSweepCollect;
    PathPlannerPath sweepCollectToBank;
    final Pose2d startingPose;
    try {
      safeFirstFuelSweep = PathPlannerPath.fromPathFile("Safe R Fuel Sweep");
      secondSweepCollect = PathPlannerPath.fromPathFile("R Sweep Collect");
      sweepCollectToBank = PathPlannerPath.fromPathFile("R Sweep to Bank");
      startingPose = safeFirstFuelSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }
    return getNeutralZoneSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        startingPose,
        safeFirstFuelSweep,
        secondSweepCollect,
        sweepCollectToBank);
  }

  private Command leftToNeutralZoneX2(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter) { // add shooter and intake later
    PathPlannerPath firstSweep;
    PathPlannerPath secondSweepCollect;
    PathPlannerPath secondSweepToBank;
    final Pose2d startingPose;
    try {
      firstSweep = PathPlannerPath.fromPathFile("L Fuel Sweep"); // this path ends at the bank
      secondSweepCollect = PathPlannerPath.fromPathFile("L Sweep Collect");
      secondSweepToBank = PathPlannerPath.fromPathFile("L Sweep to Bank");
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getNeutralZoneSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        startingPose,
        firstSweep,
        secondSweepCollect,
        secondSweepToBank);
  }

  private Command safeLeftToNeutralZoneX2(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter) { // add shooter and intake later
    PathPlannerPath safeFirstFuelSweep;
    PathPlannerPath secondSweepCollect;
    PathPlannerPath secondSweepToBank;
    final Pose2d startingPose;
    try {
      safeFirstFuelSweep =
          PathPlannerPath.fromPathFile("Safe L Fuel Sweep"); // this path ends at the bank
      secondSweepCollect = PathPlannerPath.fromPathFile("L Sweep Collect");
      secondSweepToBank = PathPlannerPath.fromPathFile("L Sweep to Bank");
      startingPose = safeFirstFuelSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getNeutralZoneSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        startingPose,
        safeFirstFuelSweep,
        secondSweepCollect,
        secondSweepToBank);
  }

  private Command rightNeutralZoneAndOutpost(
      SwerveDrivetrain drivetrain, Hopper hopper, Intake intake, Shooter shooter) {
    PathPlannerPath firstSweep;
    PathPlannerPath sweepCollect;
    PathPlannerPath sweepToOutpost;
    PathPlannerPath bankToOutpost;
    final Pose2d startingPose;

    try {
      firstSweep = PathPlannerPath.fromPathFile("R Fuel Sweep");
      sweepCollect = PathPlannerPath.fromPathFile("R Sweep Collect");
      sweepToOutpost = PathPlannerPath.fromPathFile("R Sweep to Outpost");
      bankToOutpost = PathPlannerPath.fromPathFile("R Bank to Outpost");
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    return Commands.sequence(
            Commands.runOnce(matchTimer::restart),
            setStartingPoseForAuto(startingPose, drivetrain),
            Commands.parallel(
                intake.getDeployAndStartCommand(), AutoBuilder.followPath(firstSweep)),
            getUnloadHopperCommand(hopper, intake, shooter, true),
            Commands.runOnce(hopper::stop, hopper),
            Commands.either(
                Commands.sequence(
                    AutoBuilder.followPath(sweepCollect),
                    AutoBuilder.followPath(sweepToOutpost),
                    getUnloadHopperCommand(hopper, intake, shooter, false)),
                Commands.sequence(
                    AutoBuilder.followPath(bankToOutpost),
                    getUnloadHopperCommand(hopper, intake, shooter, false)),
                () -> matchTimer.get() < 10.0))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
            });
  }

  // DEPRECATED FOR NOW
  private Command leftNeutralZoneAndDepot(
      SwerveDrivetrain drivetrain, Hopper hopper, Intake intake, Shooter shooter) {
    PathPlannerPath driveToNeutralZoneAndBack;
    PathPlannerPath driveToDepot;
    PathPlannerPath intakeFromDepot;
    PathPlannerPath leaveDepot;
    final Pose2d startingPose;
    try {
      driveToNeutralZoneAndBack = PathPlannerPath.fromPathFile("L Fuel Sweep");
      driveToDepot = PathPlannerPath.fromPathFile("Left Bank to Depot");
      intakeFromDepot = PathPlannerPath.fromPathFile("Intake Depot");
      leaveDepot = PathPlannerPath.fromPathFile("Leave Depot");
      startingPose = driveToNeutralZoneAndBack.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);

      return Commands.none();
    }

    // consider not shooting at the bank and going straight to the depot
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  Pose2d pose = startingPose;
                  if (drivetrain.shouldFlipAutoPath()) {
                    pose = FlippingUtil.flipFieldPose(startingPose);
                  }
                  drivetrain.resetPose(pose);
                }),
            Commands.parallel(
                intake.getDeployAndStartCommand(),
                AutoBuilder.followPath(driveToNeutralZoneAndBack)),
            getUnloadHopperCommand(hopper, intake, shooter, true),
            Commands.runOnce(hopper::stop, hopper),
            AutoBuilder.followPath(driveToDepot),
            AutoBuilder.followPath(intakeFromDepot),
            AutoBuilder.followPath(leaveDepot),
            getUnloadHopperCommand(hopper, intake, shooter, true))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
            });
  }
}
