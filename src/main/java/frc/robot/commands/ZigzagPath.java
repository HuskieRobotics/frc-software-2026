package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Timer;

public class ZigzagPath extends Command {
  /**
   * This Command will move the robot in one section of a zigzag path (essentially one diagonal
   * line). Multiple calls of this command will move the robot in a full zigzag pattern, composed of
   * many different smaller lines.
   *
   * <p>Requires: the Drivetrain subsystem
   */
  private final SwerveDrivetrain drivetrain;

  private double r;
  private double theta;
  private Pose2d currentPos;
  private Pose2d targetPos;
  private boolean isAttargetPos;
  Timer timer;
  private double initX;
  private double initY;
  private Rotation2d rotation;

  /**
   * @param drivetrain the drivetrain subsystem that this Command requires.
   * @param r the radius value of the line between robot's target position from its current
   *     position.
   * @param theta MUST BE IN RADIANS; the theta value of the difference between the positive x-axis
   *     and the rotation of the vector composed by the robot's initial and target position.
   * @param currentPos
   * @param targetPos
   * @param isAttargetPos
   */
  public ZigzagPath(
      SwerveDrivetrain drivetrain,
      double r,
      double theta,
      Pose2d currentPos,
      Pose2d targetPos,
      boolean isAttargetPos) {
    this.drivetrain = drivetrain;
    this.r = r;
    this.theta = theta;
    // this.currentPos = currentPos;
    // this.targetPos = targetPos;
    this.isAttargetPos = isAttargetPos;

    // addRequirements(this.drivetrain); // line has an error
    this.timer = new Timer();
  }

  /**
   * This method runs once and it runs at the start of the Zigzag command call. It starts the timer
   * and every time the initialize function is called, it is the start of a new path in the zigzag.
   * A polar axis will be created and the robot's current position will be assumed to be (0,0). The
   * robot's target position will be provided through the (r,theta) values.
   */
  @Override
  public void initialize() {
    // this.timer.restart(); // line has an error
    this.isAttargetPos = false;

    /** Convert (r,theta) values to rectangular coordinates (x,y) x = rcostheta y = rsintheta */
    this.initX = (r * (Math.cos(theta)));
    this.initY = (r * (Math.sin(theta)));
    this.rotation = new Rotation2d(initX, initY);

    this.currentPos = new Pose2d(initX, initY, rotation);
  }

  @Override
  public void execute() {
    // calculate the pose difference in the frame of the field
    Transform2d fieldRelativeDifference =
        new Transform2d(
            targetPos.getX() - currentPos.getX(),
            targetPos.getY() - currentPose.getY(),
            rotation.fromRadians(targetPos.getRotation() - currentPos.getRotation()));

    // Just calculated the pose difference between the current and target pose; now, transform the
    // current pose into the frame of the target pose?
  }

  // mathematically implement a way to convert polar coordinates (r, theta) to rectangular
  // coordinates (x,y) for the pose
  // also based on the current pose (x,y) coordinates, construct a vector between the current and
  // target pose which is what the robot will follow

}
