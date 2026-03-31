package frc.robot.commands;

import static frc.robot.subsystems.intake.IntakeConstants.JOSTLE_INITIAL_FUEL_COUNT;
import static frc.robot.subsystems.intake.IntakeConstants.JOSTLE_SUBSEQUENT_RETRACT_POSITION_METERS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterModes;
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
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // Worlds autos:
    /*
     * Left Trench Sweep: either turn or no-turn depending on chosen paths from testing
     * Left Trench-Bump Sweep: comes back over the bump both sweeps
     * Right Trench Sweep: either turn or no-turn depending on chosen paths from testing
     * Right Trench-Bump Sweep: comes back over the bump both sweeps
     * Left Support Sweep: starts behind, shoots preloads, sweeps, comes back over bump, goes depot
     * Right Sweep and Outpost
     * Outpost and Depot
     */

    autoChooser.addOption(
        "Left Trench Sweep", leftTrenchDoubleSweep(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Left Trench-Bump Sweep",
        leftTrenchBumpDoubleSweep(drivetrain, hopper, intake, shooter, shooterModes));

    autoChooser.addOption(
        "Right Trench Sweep", rightTrenchDoubleSweep(drivetrain, hopper, intake, shooter));

    autoChooser.addOption(
        "Right Trench-Bump Sweep",
        rightTrenchBumpDoubleSweep(drivetrain, hopper, intake, shooter, shooterModes));

    autoChooser.addOption(
        "Right Sweep and Outpost",
        rightNeutralZoneSweepAndOutpost(drivetrain, hopper, intake, shooter, shooterModes));

    autoChooser.addOption(
        "Left Support Sweep", leftSupportSweep(drivetrain, hopper, intake, shooter, shooterModes));

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

    // /************ Distance Test ************
    //  *
    //  * used for empirically determining the wheel radius
    //  *
    //  */
    // autoChooser.addOption(
    //     "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    // autoChooser.addOption(
    //     "Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    // autoChooser.addOption(
    //     "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    // /************ Auto Tuning ************
    //  *
    //  * useful for tuning the autonomous PID controllers
    //  *
    //  */
    // autoChooser.addOption(
    //     "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    // autoChooser.addOption(
    //     "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    // autoChooser.addOption(
    //     "Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    // autoChooser.addOption(
    //     "Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    // /************ Drive Velocity Tuning ************
    //  *
    //  * useful for tuning the drive velocity PID controller
    //  *
    //  */
    // autoChooser.addOption("Drive Velocity Tuning",
    // this.getDriveVelocityTuningCommand(drivetrain));

    // /************ Swerve Rotation Tuning ************
    //  *
    //  * useful for tuning the swerve module rotation PID controller
    //  *
    //  */
    // autoChooser.addOption(
    //     "Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    // /************ Drive Wheel Radius Characterization ************
    //  *
    //  * useful for characterizing the drive wheel Radius
    //  *
    //  */
    // autoChooser.addOption( // start by driving slowing in a circle to align wheels
    //     "Drive Wheel Radius Characterization",
    //     this.getDriveWheelRadiusCharacterizationCommand(drivetrain));
  }

  public void configureAutoCommands(DifferentialDrivetrain drivetrain) {
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
                Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain))));
  }

  private Command getSwerveRotationTuningCommand(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain))));
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
                hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocityRPS),
                getAutoJostleCommand(intake, shooter))
            .until(
                () ->
                    (checkForFuel && hopperUnloadTimer.get() > 3.0 && !shooter.getFuelDetected())),
        Commands.runOnce(hopper::stop, hopper),
        Commands.runOnce(intake::deployIntake, intake));
  }

  private Command getUnloadHopperAtOutpostCommand(
      Hopper hopper, Intake intake, Shooter shooter, boolean fullHopperAtOutpost) {

    return Commands.sequence(
        Commands.parallel(
            hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocityRPS),
            getOutpostJostleSequence(intake, shooter, fullHopperAtOutpost)));
  }

  // if empty at outpost:
  // fill up and normal
  // if full at outpost:
  // shoot for x seconds and then start jostling (LEDs to request fuel!?)
  private Command getAutoJostleCommand(Intake intake, Shooter shooter) {
    return Commands.sequence(
            Commands.runOnce(shooter::resetFuelCount),
            Commands.waitUntil(() -> shooter.getFuelCount() >= JOSTLE_INITIAL_FUEL_COUNT)
                .withTimeout(1.5),
            CrossSubsystemsCommandsFactory.getForceJostleCommand(intake))
        .withName("Auto Jostle");
  }

  // if hopper is full:
  // in parallel, shoot, and:
  // sequence: jostle for 4 seconds
  //           deploy intake
  //           wait 1 second
  //           another jostle command

  // if hopper is "empty":
  // in parallel, shoot, and:
  // sequence: wait 2 seconds for balls to enter
  //           jostle

  // can add LED triggers as well if time allows
  private Command getOutpostJostleSequence(Intake intake, Shooter shooter, boolean fullHopper) {
    return Commands.either(
        Commands.sequence(
            getAutoJostleCommand(intake, shooter).withTimeout(4.0),
            // redeploy intake and allow for more fuel entry. can flash leds here too
            Commands.runOnce(intake::deployIntake),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(
                    () -> LEDs.getInstance().requestState(LEDs.States.OUTPOST_FLASH_AUTO))),
            getAutoJostleCommand(intake, shooter)),
        Commands.sequence(
            // wait 2s to allow for fuel entry can flash LEDs here as well
            Commands.waitSeconds(2.0), getAutoJostleCommand(intake, shooter)),
        () -> fullHopper);
  }

  private Command getTrenchDoubleSweepAuto(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      Pose2d startingPose,
      PathPlannerPath firstSweep,
      PathPlannerPath secondSweep) {

    return Commands.sequence(
            Commands.runOnce(matchTimer::restart),
            setStartingPoseForAuto(startingPose, drivetrain),
            Commands.parallel(
                intake.getDeployAndStartInAutoCommand(), AutoBuilder.followPath(firstSweep)),
            getUnloadHopperCommand(hopper, intake, shooter, true).withTimeout(5.0),
            Commands.runOnce(hopper::stop, hopper),
            Commands.runOnce(intake::deployIntake),
            AutoBuilder.followPath(secondSweep),
            getUnloadHopperCommand(hopper, intake, shooter, false))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
              intake.startRoller();
            });
  }

  private Command getTrenchBumpDoubleSweepAuto(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes,
      Pose2d startingPose,
      PathPlannerPath firstSweep,
      PathPlannerPath slowToTrench,
      PathPlannerPath secondSweep) {

    return Commands.sequence(
            Commands.runOnce(matchTimer::restart),
            setStartingPoseForAuto(startingPose, drivetrain),
            Commands.parallel(
                intake.getDeployAndStartInAutoCommand(), AutoBuilder.followPath(firstSweep)),
            Commands.runOnce(shooterModes::enableShootOnTheMoveInAuto),
            Commands.deadline(
                AutoBuilder.followPath(slowToTrench),
                hopper.getFeedFuelIntoShooterCommand(shooter::getFlywheelLeadVelocityRPS),
                getAutoJostleCommand(intake, shooter)),
            Commands.runOnce(shooterModes::disableShootOnTheMoveInAuto),
            Commands.runOnce(hopper::stop, hopper), // could be unnecessary
            Commands.runOnce(intake::deployIntake),
            AutoBuilder.followPath(secondSweep),
            getUnloadHopperCommand(hopper, intake, shooter, false))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
              intake.startRoller();
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

  private Command leftTrenchDoubleSweep(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter) { // add shooter and intake later
    PathPlannerPath firstSweep;
    PathPlannerPath secondSweep;
    final Pose2d startingPose;
    try {
      // exchange with turn vs. no-turn for testing
      firstSweep = PathPlannerPath.fromPathFile("L No-Turn Fuel Sweep");
      secondSweep = PathPlannerPath.fromPathFile("L No-Turn Second Collect");
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getTrenchDoubleSweepAuto(
        drivetrain, hopper, intake, shooter, startingPose, firstSweep, secondSweep);
  }

  private Command rightTrenchDoubleSweep(
      SwerveDrivetrain drivetrain, Hopper hopper, Intake intake, Shooter shooter) {
    PathPlannerPath firstSweep;
    PathPlannerPath secondSweep;
    final Pose2d startingPose;
    try {
      // exchange with turn vs. no-turn for testing
      firstSweep = PathPlannerPath.fromPathFile("L No-Turn Fuel Sweep").mirrorPath();
      secondSweep = PathPlannerPath.fromPathFile("L No-Turn Second Collect").mirrorPath();
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getTrenchDoubleSweepAuto(
        drivetrain, hopper, intake, shooter, startingPose, firstSweep, secondSweep);
  }

  private Command leftTrenchBumpDoubleSweep(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes) {
    PathPlannerPath firstSweep;
    PathPlannerPath secondSweep;
    PathPlannerPath slowToTrench;
    final Pose2d startingPose;
    try {
      firstSweep = PathPlannerPath.fromPathFile("L Fuel Sweep Over Bump");
      slowToTrench = PathPlannerPath.fromPathFile("L Bump to Trench");
      secondSweep = PathPlannerPath.fromPathFile("L Bump Second Collect");
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getTrenchBumpDoubleSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        shooterModes,
        startingPose,
        firstSweep,
        slowToTrench,
        secondSweep);
  }

  private Command rightTrenchBumpDoubleSweep(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes) {
    PathPlannerPath firstSweep;
    PathPlannerPath secondSweep;
    PathPlannerPath slowToTrench;
    final Pose2d startingPose;
    try {
      firstSweep = PathPlannerPath.fromPathFile("L Fuel Sweep Over Bump").mirrorPath();
      slowToTrench = PathPlannerPath.fromPathFile("L Bump to Trench").mirrorPath();
      secondSweep = PathPlannerPath.fromPathFile("L Bump Second Collect").mirrorPath();
      startingPose = firstSweep.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return getTrenchBumpDoubleSweepAuto(
        drivetrain,
        hopper,
        intake,
        shooter,
        shooterModes,
        startingPose,
        firstSweep,
        slowToTrench,
        secondSweep);
  }

  private Command leftSupportSweep(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes) {
    PathPlannerPath firstSweepToDepot;
    PathPlannerPath intakeDepot;
    PathPlannerPath leaveDepot;
    final Pose2d startingPose;
    try {
      firstSweepToDepot = PathPlannerPath.fromPathFile("L Support Collect");
      intakeDepot = PathPlannerPath.fromPathFile("Intake Depot");
      leaveDepot = PathPlannerPath.fromPathFile("Leave Depot");
      startingPose = firstSweepToDepot.getStartingHolonomicPose().orElseThrow();
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file.");
      pathFileMissingAlert.set(true);
      return Commands.none();
    }

    return Commands.sequence(
        // shoot preloads
        getUnloadHopperCommand(hopper, intake, shooter, true).withTimeout(2.5),
        Commands.runOnce(hopper::stop, hopper),
        Commands.runOnce(intake::deployIntake),
        AutoBuilder.followPath(firstSweepToDepot),
        getUnloadHopperCommand(hopper, intake, shooter, true).withTimeout(4.0),
        Commands.runOnce(hopper::stop, hopper),
        Commands.runOnce(intake::deployIntake),
        AutoBuilder.followPath(intakeDepot),
        AutoBuilder.followPath(leaveDepot),
        getUnloadHopperCommand(hopper, intake, shooter, false));
  }

  private Command rightNeutralZoneSweepAndOutpost(
      SwerveDrivetrain drivetrain,
      Hopper hopper,
      Intake intake,
      Shooter shooter,
      ShooterModes shooterModes) {
    PathPlannerPath firstSweep;
    PathPlannerPath outpostToMid;
    final Pose2d startingPose;

    try {
      firstSweep = PathPlannerPath.fromPathFile("R Fuel Sweep to Outpost");
      outpostToMid = PathPlannerPath.fromPathFile("Outpost to Mid");
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
                intake.getDeployAndStartInAutoCommand(), AutoBuilder.followPath(firstSweep)),
            getUnloadHopperAtOutpostCommand(hopper, intake, shooter, true)
                .until(() -> (matchTimer.get() > 17.5)),
            Commands.runOnce(shooterModes::enableShootOnTheMoveInAuto),
            Commands.runOnce(
                () -> intake.setLinearPosition(JOSTLE_SUBSEQUENT_RETRACT_POSITION_METERS)),
            AutoBuilder.followPath(outpostToMid))
        .finallyDo(
            () -> {
              hopper.stop();
              intake.deployIntake();
              intake.startRoller();
              shooterModes.disableShootOnTheMoveInAuto();
            });
  }

  public double getCustomMatchTime() {
    return matchTimer.get();
  }
}
