package frc.robot.operator_interface;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;
import java.util.Random;
import java.util.concurrent.TimeUnit;

/**
 * OperatorDashboard is a class that implements the OperatorInterface. It is not a joystick,
 * controller, or physical button panel. Instead, it is a software-based dashboard with virtual
 * buttons. It is designed to be used by the operator on a handheld touchscreen running Elastic. For
 * a more sophisticated example, including implementing groups of buttons where only one may be
 * selected at a time, refer to the 2025 Huskie Robotics code base.
 */
public class OperatorDashboard implements OperatorInterface {
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

  public final LoggedTunableBoolean practiceMatchTimerRunning =
      new LoggedTunableBoolean("operatorDashboard/Practice Match Timer Running", false, true);

  public final LoggedTunableBoolean practiceMatchTimerReset =
      new LoggedTunableBoolean("operatorDashboard/Practice Match Timer Reset", false, true);

  private String selectedAlliance = "Red";

  private static final double MATCH_TIME_SECONDS = 160;
  private static final double AUTO_END_TIME = 140;
  private static final double TRANSITION_END_TIME = 130;
  private static final double SHIFT_1_END_TIME = 105;
  private static final double SHIFT_2_END_TIME = 80;
  private static final double SHIFT_3_END_TIME = 55;
  private static final double SHIFT_4_END_TIME = 30;
  private static final double END_GAME_END_TIME = 0;

  private double practiceMatchStartTime = 160;
  private double pausedRemainingTime = MATCH_TIME_SECONDS;
  private boolean isTimerRunning = false;
  private boolean lastTimerRunningState = false;
  private boolean lastTimerResetState = false;
  private boolean hasAutoEnded = false;
  private boolean isRedAlliance = false;
  private final Random random = new Random();

  private final DoublePublisher timerPublisher;
  private final StringPublisher shiftTimerPublisher;
  private final StringPublisher currentShiftPublisher;
  private final StringPublisher activeAlliancePublisher;
  private final StringPublisher autosWinnerPublisher;
  private final StringPublisher selectedAlliancePublisher;
  private final StringArrayPublisher selectedAllianceOptionsPublisher;

  public OperatorDashboard() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    timerPublisher = nt.getDoubleTopic("/SmartDashboard/Practice Match Timer").publish();
    shiftTimerPublisher = nt.getStringTopic("/SmartDashboard/Shift Timer").publish();
    currentShiftPublisher = nt.getStringTopic("/SmartDashboard/Current Shift").publish();
    activeAlliancePublisher = nt.getStringTopic("/SmartDashboard/Active Alliance").publish();
    autosWinnerPublisher = nt.getStringTopic("/SmartDashboard/Autos Winner").publish();
    selectedAlliancePublisher = nt.getStringTopic("/SmartDashboard/Selected Alliance").publish();
    selectedAllianceOptionsPublisher = nt.getStringArrayTopic("/SmartDashboard/Selected Alliance/options").publish();

    resetTimer();
    updateSelectedAllianceDisplay();
    selectedAllianceOptionsPublisher.set(new String[] {"Red", "Blue"});
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
    return new Trigger(() -> practiceMatchTimerRunning.get());
  }

  @Override
  public Trigger getPausePracticeMatchTrigger() {
    return new Trigger(() -> practiceMatchTimerRunning.get());
  }

  @Override
  public Trigger getResetPracticeMatchTrigger() {
    return new Trigger(() -> practiceMatchTimerReset.get());
  }

  public void updateTimer() {
    boolean currentRunningState = practiceMatchTimerRunning.get();
    boolean currentResetState = practiceMatchTimerReset.get();

    if (currentRunningState && !lastTimerRunningState) {
      if (isTimerRunning) {
        pauseTimer();
      } else {
        startTimer();
      }
    }
    lastTimerRunningState = currentRunningState;

    if (currentResetState && !lastTimerResetState) {
      resetTimer();
    }
    lastTimerResetState = currentResetState;

    if (!isTimerRunning) {
      timerPublisher.set(pausedRemainingTime);
      updateShiftDisplay(pausedRemainingTime);
      return;
    }

    double elapsed = Math.abs(Timer.getFPGATimestamp() - practiceMatchStartTime);
    double remaining = Math.max(0.0, MATCH_TIME_SECONDS - elapsed);
    timerPublisher.set(remaining);

    updateShiftDisplay(remaining);

    if (!hasAutoEnded && remaining <= AUTO_END_TIME) {
      hasAutoEnded = true;
      isRedAlliance = random.nextBoolean();
      String autoWinner = isRedAlliance ? "Red" : "Blue";
      autosWinnerPublisher.set(autoWinner);
    }

    if (remaining <= 0.0) {
      isTimerRunning = false;
      pausedRemainingTime = 0.0;
    }
  }

  private void updateShiftDisplay(double remainingTime) {
    String shiftName;
    double shiftEndTime;
    
    if(remainingTime > 0 && remainingTime <= AUTO_END_TIME) {
      shiftName = "AUTO";
      shiftEndTime = AUTO_END_TIME;
    }
    else if (remainingTime > AUTO_END_TIME && remainingTime <= TRANSITION_END_TIME) {
      shiftName = "TRANSITION";
      shiftEndTime = TRANSITION_END_TIME;
    } else if (remainingTime > TRANSITION_END_TIME && remainingTime <= SHIFT_1_END_TIME) {
      shiftName = "SHIFT 1";
      shiftEndTime = SHIFT_1_END_TIME;
    } else if (remainingTime > SHIFT_1_END_TIME && remainingTime <= SHIFT_2_END_TIME) {
      shiftName = "SHIFT 2";
      shiftEndTime = SHIFT_2_END_TIME;
    } else if (remainingTime > SHIFT_2_END_TIME && remainingTime <= SHIFT_3_END_TIME) {
      shiftName = "SHIFT 3";
      shiftEndTime = SHIFT_3_END_TIME;
    } else if (remainingTime > SHIFT_3_END_TIME && remainingTime <= SHIFT_4_END_TIME) {
      shiftName = "SHIFT 4";
      shiftEndTime = SHIFT_4_END_TIME;
    } else if (remainingTime > SHIFT_4_END_TIME && remainingTime <= END_GAME_END_TIME) {
      shiftName = "END GAME";
      shiftEndTime = END_GAME_END_TIME;
    } else {
      shiftName = "MATCH ENDED";
      shiftEndTime = 0;
    }

    double shiftTimeElapsed = getShiftStartBoundary(shiftName) - remainingTime;
    double shiftTimeRemaining = shiftEndTime - shiftTimeElapsed;

    currentShiftPublisher.set(shiftName);
    shiftTimerPublisher.set(formatTime(Math.max(0.0, shiftTimeRemaining)));
    updateActiveAlliance(remainingTime);
  }

  private String formatTime(double seconds) {
    int totalSeconds = (int) Math.round(seconds);
    int minutes = (int) TimeUnit.SECONDS.toMinutes(totalSeconds);
    int secs = totalSeconds - (minutes * 60);
    return String.format("%02d:%02d", minutes, secs);
  }

  private double getShiftStartBoundary(String shiftName) {
    switch (shiftName) {
      case "AUTO":
        return MATCH_TIME_SECONDS;
      case "TRANSITION":
        return AUTO_END_TIME + 1;
      case "SHIFT 1":
        return TRANSITION_END_TIME + 1;
      case "SHIFT 2":
        return SHIFT_1_END_TIME + 1;
      case "SHIFT 3":
        return SHIFT_2_END_TIME + 1;
      case "SHIFT 4":
        return SHIFT_3_END_TIME + 1;
      case "END GAME":
        return SHIFT_4_END_TIME + 1;
      default:
        return 0;
    }
  }

  private void updateActiveAlliance(double remainingTime) {
    if (!hasAutoEnded) {

      activeAlliancePublisher.set("Both");
      return;
    }

    boolean redActive;

    if (remainingTime > TRANSITION_END_TIME) {
      activeAlliancePublisher.set("Both");
      return;
    } else if (remainingTime > SHIFT_1_END_TIME) {
      redActive = !isRedAlliance;
    } else if (remainingTime > SHIFT_2_END_TIME) {
      redActive = isRedAlliance;
    } else if (remainingTime > SHIFT_3_END_TIME) {
      redActive = !isRedAlliance;
    } else if (remainingTime > SHIFT_4_END_TIME) {
      redActive = isRedAlliance;
    } else {
      activeAlliancePublisher.set("Both");
      return;
    }

    boolean userSelectedRed = "Red".equals(selectedAlliance);
    if (userSelectedRed) {
      activeAlliancePublisher.set(redActive ? "Red (YOU)" : "Blue");
    } else {
      activeAlliancePublisher.set(redActive ? "Red" : "Blue (YOU)");
    }
  }

  public void setSelectedAlliance(String alliance) {
    if ("Red".equals(alliance) || "Blue".equals(alliance)) {
      selectedAlliance = alliance;
      updateSelectedAllianceDisplay();
    }
  }

  private void updateSelectedAllianceDisplay() {
    selectedAlliancePublisher.set(selectedAlliance);
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
      pausedRemainingTime = Math.max(0.0, Math.abs(MATCH_TIME_SECONDS - elapsed));
      isTimerRunning = false;
    }
  }

  public void resetTimer() {
    isTimerRunning = false;
    pausedRemainingTime = MATCH_TIME_SECONDS;
    hasAutoEnded = false;
    isRedAlliance = true;
    timerPublisher.set(MATCH_TIME_SECONDS);
    currentShiftPublisher.set("AUTO");
    shiftTimerPublisher.set(formatTime(AUTO_END_TIME));
    activeAlliancePublisher.set("Both");
    autosWinnerPublisher.set("-");
  }
}
