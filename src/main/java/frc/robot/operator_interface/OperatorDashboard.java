package frc.robot.operator_interface;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;
import java.util.Random;

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
  private final StringSubscriber selectedAllianceSubscriber;
  private final StringArrayPublisher selectedAllianceOptionsPublisher;

  public OperatorDashboard() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    timerPublisher = nt.getDoubleTopic("/SmartDashboard/Practice Match Timer").publish();
    shiftTimerPublisher = nt.getStringTopic("/SmartDashboard/Shift Timer").publish();
    currentShiftPublisher = nt.getStringTopic("/SmartDashboard/Current Shift").publish();
    activeAlliancePublisher = nt.getStringTopic("/SmartDashboard/Active Alliance").publish();
    autosWinnerPublisher = nt.getStringTopic("/SmartDashboard/Autos Winner").publish();
    selectedAlliancePublisher = nt.getStringTopic("/SmartDashboard/Selected Alliance").publish();
    selectedAllianceSubscriber =
        nt.getStringTopic("/SmartDashboard/Selected Alliance").subscribe("Red");
    selectedAllianceOptionsPublisher =
        nt.getStringArrayTopic("/SmartDashboard/Selected Alliance/options").publish();

    // Set options FIRST before setting the selected value
    selectedAllianceOptionsPublisher.set(new String[] {"Red", "Blue"});
    resetTimer();
    updateSelectedAllianceDisplay();
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

    // Read Elastic's alliance selection and update our internal state
    String elasticSelection = selectedAllianceSubscriber.get();
    if ("Red".equals(elasticSelection) || "Blue".equals(elasticSelection)) {
      selectedAlliance = elasticSelection;
    }

    // ALWAYS publish selected alliance every cycle (keeps Elastic happy)
    selectedAlliancePublisher.set(selectedAlliance);
    selectedAllianceOptionsPublisher.set(new String[] {"Red", "Blue"});

    // Handle start/stop button edge detection
    if (currentRunningState && !lastTimerRunningState) {
      if (isTimerRunning) {
        pauseTimer();
      } else {
        // Start timer - capture timestamp ONCE and use it for everything
        double now = Timer.getFPGATimestamp();
        practiceMatchStartTime = now - (MATCH_TIME_SECONDS - pausedRemainingTime);
        isTimerRunning = true;
        // Immediately calculate and update with current time - no delay
        double elapsed = now - practiceMatchStartTime;
        double remaining = Math.max(0.0, MATCH_TIME_SECONDS - elapsed);
        timerPublisher.set(remaining);
        updateShiftDisplay(remaining, true);
        lastTimerRunningState = currentRunningState;
        return;
      }
    }
    lastTimerRunningState = currentRunningState;

    // Handle reset button edge detection
    if (currentResetState && !lastTimerResetState) {
      resetTimer();
    }
    lastTimerResetState = currentResetState;

    // If timer is paused, display paused time
    if (!isTimerRunning) {
      timerPublisher.set(pausedRemainingTime);
      updateShiftDisplay(pausedRemainingTime, false);
      return;
    }

    // Timer is running - calculate remaining time
    double now = Timer.getFPGATimestamp();
    double elapsed = now - practiceMatchStartTime;
    double remaining = MATCH_TIME_SECONDS - elapsed;

    // Check for auto end
    if (!hasAutoEnded && remaining <= AUTO_END_TIME) {
      hasAutoEnded = true;
      isRedAlliance = random.nextBoolean();
      autosWinnerPublisher.set(isRedAlliance ? "Red" : "Blue");
    }

    // Check for match end
    if (remaining <= 0.0) {
      remaining = 0.0;
      isTimerRunning = false;
      pausedRemainingTime = 0.0;
    }

    timerPublisher.set(remaining);
    updateShiftDisplay(remaining, true);
  }

  private void updateShiftDisplay(double remainingTime, boolean timerRunning) {
    String shiftName;
    double shiftLowerBound;

    if (remainingTime > AUTO_END_TIME) {
      shiftName = "AUTO";
      shiftLowerBound = AUTO_END_TIME;
    } else if (remainingTime > TRANSITION_END_TIME) {
      shiftName = "TRANSITION";
      shiftLowerBound = TRANSITION_END_TIME;
    } else if (remainingTime > SHIFT_1_END_TIME) {
      shiftName = "SHIFT 1";
      shiftLowerBound = SHIFT_1_END_TIME;
    } else if (remainingTime > SHIFT_2_END_TIME) {
      shiftName = "SHIFT 2";
      shiftLowerBound = SHIFT_2_END_TIME;
    } else if (remainingTime > SHIFT_3_END_TIME) {
      shiftName = "SHIFT 3";
      shiftLowerBound = SHIFT_3_END_TIME;
    } else if (remainingTime > SHIFT_4_END_TIME) {
      shiftName = "SHIFT 4";
      shiftLowerBound = SHIFT_4_END_TIME;
    } else if (remainingTime > END_GAME_END_TIME) {
      shiftName = "END GAME";
      shiftLowerBound = END_GAME_END_TIME;
    } else {
      shiftName = "MATCH ENDED";
      shiftLowerBound = 0;
    }

    double shiftTimeRemaining = remainingTime - shiftLowerBound;
    int totalSeconds = (int) Math.round(Math.max(0.0, shiftTimeRemaining));
    int minutes = totalSeconds / 60;
    int secs = totalSeconds % 60;

    currentShiftPublisher.set(shiftName);
    shiftTimerPublisher.set(String.format("%02d:%02d", minutes, secs));

    // Update active alliance display
    updateActiveAlliance(remainingTime);
  }

  private void updateActiveAlliance(double remainingTime) {
    // Before auto ends or during transition, show both alliances
    if (!hasAutoEnded || remainingTime > TRANSITION_END_TIME) {
      activeAlliancePublisher.set("Both");
      return;
    }

    // Determine which alliance is active for this shift
    boolean redIsActive;
    if (remainingTime > SHIFT_1_END_TIME) {
      redIsActive = !isRedAlliance;
    } else if (remainingTime > SHIFT_2_END_TIME) {
      redIsActive = isRedAlliance;
    } else if (remainingTime > SHIFT_3_END_TIME) {
      redIsActive = !isRedAlliance;
    } else if (remainingTime > SHIFT_4_END_TIME) {
      redIsActive = isRedAlliance;
    } else {
      activeAlliancePublisher.set("Both");
      return;
    }

    // Show which alliance is active and mark the user's selected alliance
    boolean userSelectedRed = "Red".equals(selectedAlliance);
    if (redIsActive) {
      // Red alliance is active
      activeAlliancePublisher.set(userSelectedRed ? "Red (YOU)" : "Red");
    } else {
      // Blue alliance is active
      activeAlliancePublisher.set(userSelectedRed ? "Blue (YOU)" : "Blue");
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
    updateShiftDisplay(MATCH_TIME_SECONDS, false);
    activeAlliancePublisher.set("Both");
    autosWinnerPublisher.set("-");
  }
}
