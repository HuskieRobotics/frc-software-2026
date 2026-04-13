package frc.robot.operator_interface;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;

/**
 * OperatorDashboard is a class that implements the OperatorInterface. It is not a joystick,
 * controller, or physical button panel. Instead, it is a software-based dashboard with virtual
 * buttons. It is designed to be used by the operator on a handheld touchscreen running Elastic. For
 * a more sophisticated example, including implementing groups of buttons where only one may be
 * selected at a time, refer to the 2025 Huskie Robotics code base.
 */
public class OperatorDashboard implements OperatorInterface {
  // the readAndWrite argument must be true or else the dashboard will not work when tuning mode is
  // disabled.
  public final LoggedTunableBoolean enableVision =
      new LoggedTunableBoolean("operatorDashboard/Enable Vision", true, true);

  public final LoggedTunableBoolean passToggle =
      new LoggedTunableBoolean("operatorDashboard/Pass", false, true);

  // public final LoggedTunableBoolean lockTurretForBankToggle =
  //     new LoggedTunableBoolean("operatorDashboard/Lock Turret For Bank", false, true);

  public final LoggedTunableBoolean shootOnTheMoveToggle =
      new LoggedTunableBoolean("operatorDashboard/Shoot On The Move", false, true);

  public final LoggedTunableBoolean passOnTheMoveToggle =
      new LoggedTunableBoolean("operatorDashboard/Pass On The Move", true, true);

  // public final LoggedTunableBoolean objectDetectionToggle =
  //     new LoggedTunableBoolean("operatorDashboard/Object Detection Enabled", true, true);

  public final LoggedTunableBoolean hubActiveAtHomeToggle =
      new LoggedTunableBoolean("operatorDashboard/Hub Active (Home)", false, true);

  public final LoggedTunableBoolean lockShooterToggle =
      new LoggedTunableBoolean("operatorDashboard/Lock Shooter", false, true);

  public final LoggedTunableBoolean autoSnapsEnabled =
      new LoggedTunableBoolean("operatorDashboard/Auto Snaps Enabled", true, true);

  public final LoggedTunableBoolean slowShooterForPitTest =
      new LoggedTunableBoolean("operatorDashboard/Slow Shooter For Pit Test", false, true);

  public final LoggedTunableBoolean startPracticeMatch =
      new LoggedTunableBoolean("operatorDashboard/Start Practice Match", false, true);

  public final LoggedTunableBoolean pausePracticeMatch =
      new LoggedTunableBoolean("operatorDashboard/Pause Practice Match", false, true);

  public final LoggedTunableBoolean resetPracticeMatch =
      new LoggedTunableBoolean("operatorDashboard/Reset Practice Match", false, true);

  private static final double MATCH_TIME_SECONDS = 160;
  private double practiceMatchStartTime = -1.0;
  private double pausedRemainingTime = 0.0;
  private boolean isTimerRunning = false;
  private final DoublePublisher timerPublisher;

  public OperatorDashboard() {
    // Publish timer value to NetworkTables for Elastic to display
    timerPublisher =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/SmartDashboard/Practice Match Timer")
            .publish();
  }

  @Override
  public Trigger getPassToggle() {
    return new Trigger(() -> passToggle.get());
  }

  // @Override
  // public Trigger getLockTurretForBankToggle() {
  //   return new Trigger(() -> lockTurretForBankToggle.get());
  // }

  @Override
  public Trigger getShootOnTheMoveToggle() {
    return new Trigger(() -> shootOnTheMoveToggle.get());
  }

  @Override
  public Trigger getPassOnTheMoveToggle() {
    return new Trigger(() -> passOnTheMoveToggle.get());
  }

  @Override
  public Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> enableVision.get());
  }

  @Override
  public Trigger getHubActiveAtHomeToggle() {
    return new Trigger(() -> (!DriverStation.isFMSAttached() && hubActiveAtHomeToggle.get()));
  }

  @Override
  public Trigger getLockShooterToggle() {
    return new Trigger(() -> lockShooterToggle.get());
  }

  // @Override
  // public Trigger getObjectDetectionToggle() {
  //   return new Trigger(() -> objectDetectionToggle.get());
  // }

  @Override
  public Trigger getAutoSnapsEnabledTrigger() {
    return new Trigger(() -> autoSnapsEnabled.get());
  }

  @Override
  public Trigger getSlowShooterForPitTest() {
    return new Trigger(() -> slowShooterForPitTest.get());
  }

  @Override
  public Trigger getStartPracticeMatchTrigger() {
    return new Trigger(() -> startPracticeMatch.get());
  }

  @Override
  public Trigger getPausePracticeMatchTrigger() {
    return new Trigger(() -> pausePracticeMatch.get());
  }

  @Override
  public Trigger getResetPracticeMatchTrigger() {
    return new Trigger(() -> resetPracticeMatch.get());
  }

  public void updateTimer() {
    if (!isTimerRunning) {
      timerPublisher.set(pausedRemainingTime);
      return;
    }

    double elapsed = Timer.getFPGATimestamp() - practiceMatchStartTime;
    double remaining = Math.max(0.0, MATCH_TIME_SECONDS - elapsed);
    timerPublisher.set(remaining);

    if (remaining <= 0.0) {
      isTimerRunning = false;
      pausedRemainingTime = 0.0;
    }
  }

  public void startTimer() {
    if (!isTimerRunning) {
      practiceMatchStartTime =
          Timer.getFPGATimestamp() - (MATCH_TIME_SECONDS - pausedRemainingTime);
      isTimerRunning = true;
    }
  }

  public void pauseTimer() {
    if (isTimerRunning) {
      double elapsed = Timer.getFPGATimestamp() - practiceMatchStartTime;
      pausedRemainingTime = Math.max(0.0, MATCH_TIME_SECONDS - elapsed);
      isTimerRunning = false;
    }
  }

  public void resetTimer() {
    isTimerRunning = false;
    pausedRemainingTime = MATCH_TIME_SECONDS;
    timerPublisher.set(MATCH_TIME_SECONDS);
  }
}
