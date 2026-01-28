package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DriveToBank extends DriveToPose {

  // boost is negative to drive backwards into wall
  private static final double DRIVE_TO_BANK_X_BOOST = -0.25; // in m/s

  public DriveToBank(
      SwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPoseSupplier,
      ProfiledPIDController xController,
      ProfiledPIDController yController,
      ProfiledPIDController thetaController,
      Transform2d targetTolerance,
      boolean finishesWhenAtTarget,
      Consumer<Boolean> atTargetConsumer,
      Consumer<Transform2d> poseDifferenceConsumer,
      double timeout) {
    super(
        drivetrain,
        targetPoseSupplier,
        xController,
        yController,
        thetaController,
        targetTolerance,
        finishesWhenAtTarget,
        atTargetConsumer,
        poseDifferenceConsumer,
        timeout);
  }

  @Override
  public Translation2d adjustVelocities(
      Translation2d velocitiesInTargetFrame, Transform2d poseDifferenceInTargetFrame) {
    return new Translation2d(
        velocitiesInTargetFrame.getX() + DRIVE_TO_BANK_X_BOOST, velocitiesInTargetFrame.getY());
  }
}
