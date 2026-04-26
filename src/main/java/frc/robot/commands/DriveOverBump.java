package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the supplied pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to PathPlanner. For generating a path on the fly and
 * following that path, refer to Field2d's makePath method.
 *
 * <p>This command is highly customizable with a variety of suppliers. It can be further customized
 * by extending and overriding the desired methods.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When one of the following is true: it is specified that this command finishes when
 * the the robot is at the target pose and the robot is at the target pose (within the specified
 * tolerances), when the timeout occurs, when it is determined that the robot cannot reach the
 * target pose, or when the move-to-pose feature is disabled on the drivetrain subsystem.
 *
 * <p>At End: stops the drivetrain, disables acceleration limiting
 */
public class DriveOverBump extends DriveToPose {

  private Supplier<Pose2d> targetPoseSupplier;

  private final LoggedTunableNumber boostVelocityMPS =
      new LoggedTunableNumber("DriveOverBump/BoostVelocityMPS", 1.0);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * target pose.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param targetPoseSupplier a supplier that returns the pose to drive to. A pose supplier is
   *     specified instead of a pose since the target pose may not be known when this command is
   *     created. In addition, for more complex applications, this provides the opportunity for the
   *     target pose to change while this command executes.
   * @param xController the profiled PID controller for controlling the x position in the frame of
   *     the target pose
   * @param yController the profiled PID controller for controlling the y position in the frame of
   *     the target pose
   * @param thetaController the profiled PID controller for controlling the theta position
   * @param targetTolerance the tolerance for determining if the robot has reached the target pose
   *     specified in meters. This tolerance is specified in the frame of the target pose, not in
   *     the field frame. If more complex logic is needed, the isPoseWithinTolerance method can be
   *     overriden (e.g., for asymmetric tolerances).
   * @param finishesWhenAtTarget if true, this command will finish when the robot is at the target
   *     pose (within the specified tolerances). If false, this command will not finish when the
   *     robot is at the target pose, but will continue until another condition is met (e.g.,
   *     timeout, cannot reach target pose, move-to-pose feature disabled) or the command is
   *     canceled.
   * @param atTargetConsumer a consumer that is invoked with a boolean indicating whether the robot
   *     is at the target pose (within the specified tolerances). This can be used to change LEDs to
   *     indicate to the driver that the robot is at the target pose.
   * @param poseDifferenceConsumer a consumer that is invoked with the pose difference in the frame
   *     of the target pose. This can be used in more complicated collections of commands to control
   *     other subsystems as the robot moves towards the target pose.
   * @param timeout the timeout in seconds for this command. If the timeout is reached, this command
   *     will finish. If the timeout is less than or equal to zero, this command will not time out.
   *     This is useful for debugging purposes, but should be set to a reasonable value in
   *     production code to prevent the robot from driving indefinitely.
   */
  public DriveOverBump(
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

    this.targetPoseSupplier = targetPoseSupplier;
  }

  /**
   * This method is invoked to adjust the velocities in the frame of the target pose. It can be
   * overridden to provide custom behavior for adjusting the velocities. By default, it returns the
   * velocities unchanged. This provides the opportunity to adjust the velocities based on the pose
   * difference in the target frame. For example, this can be used to boost the x velocity to ensure
   * the robot drives into a field element.
   *
   * @param velocitiesInTargetFrame the velocities in the robot frame, calculated in the frame of
   *     the target pose
   * @param poseDifferenceInTargetFrame the pose difference in the target frame, which can be used
   *     to adjust the velocities
   * @return the adjusted velocities in the of the target pose
   */
  @Override
  public Translation2d adjustVelocities(
      Translation2d velocitiesInTargetFrame, Transform2d poseDifferenceInTargetFrame) {

    Logger.recordOutput(
        "DriveOverBump/initial x velocity (target frame)",
        velocitiesInTargetFrame.getX(),
        MetersPerSecond);
    Logger.recordOutput(
        "DriveOverBump/initial y velocity (target frame)",
        velocitiesInTargetFrame.getY(),
        MetersPerSecond);

    // adjust the velocity based on the rotation of the target pose to ensure the robot drives over
    // the bump
    Rotation2d targetRotation =
        Field2d.getInstance().getAlliance() == Alliance.Blue
            ? Rotation2d.fromDegrees(180.0)
            : Rotation2d.fromDegrees(0.0);
    targetRotation = targetRotation.minus(targetPoseSupplier.get().getRotation());

    Logger.recordOutput(
        "DriveOverBump/target rotation for boost", targetRotation.getDegrees(), Degrees);

    Translation2d boostVelocity =
        new Translation2d(boostVelocityMPS.get(), 0.0).rotateBy(targetRotation);
    velocitiesInTargetFrame = velocitiesInTargetFrame.plus(boostVelocity);

    return velocitiesInTargetFrame;
  }
}
