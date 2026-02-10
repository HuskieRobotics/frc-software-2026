package frc.robot.commands;

import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.vision.Vision;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TeleopSwerveCollectFuel extends TeleopSwerve {
  private final SwerveDrivetrain drivetrain;
  // private final Intake intake;
  private final Vision vision;

  private boolean wasFieldRelative;

  public TeleopSwerveCollectFuel(
      SwerveDrivetrain drivetrain, /* Intake intake, */
      Vision vision,
      DoubleSupplier translationXSupplier) {
    super(drivetrain, translationXSupplier, vision::getFuelAdjustment, () -> 0.0);
    this.drivetrain = drivetrain;
    // this.intake = intake;
    this.vision = vision;

    this.wasFieldRelative = true;
  }

  @Override
  public void initialize() {
    this.wasFieldRelative = drivetrain.getFieldRelative();
    drivetrain.disableFieldRelative();

    Logger.recordOutput("TeleopSwerveCollectFuel/isFinished", false);
  }

  @Override
  public void end(boolean interrupted) {
    if (wasFieldRelative) {
      drivetrain.enableFieldRelative();
    }

    Logger.recordOutput("TeleopSwerveCollectFuel/isFinished", true);
  }

  // Will rely more specifically on DriveToPose override button pressed by driver
  @Override
  public boolean isFinished() {
    // add tolerance once units/difference actually understood in PID controller
    return vision.getFuelAdjustment() == 0.0;
  }
}
