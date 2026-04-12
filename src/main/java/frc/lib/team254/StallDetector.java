package frc.lib.team254;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class StallDetector implements BooleanSupplier {

  private double velocityThresholdRPS;
  private double timeThresholdSeconds;
  private Timer velocityBelowThresholdTimer;
  private Trigger cachedTrigger;
  private boolean lastValue = false;

  public StallDetector(double velocityThresholdRPS, double timeThresholdSeconds) {
    this.velocityThresholdRPS = velocityThresholdRPS;
    this.timeThresholdSeconds = timeThresholdSeconds;
    this.velocityBelowThresholdTimer = new Timer();
  }

  public boolean update(double referenceVelocityRPS, double actualVelocityRPS) {
    boolean isStalled =
        (referenceVelocityRPS != 0) && (Math.abs(actualVelocityRPS) < velocityThresholdRPS);

    if (isStalled) {
      velocityBelowThresholdTimer.start();
      boolean newValue = velocityBelowThresholdTimer.hasElapsed(timeThresholdSeconds);
      lastValue = newValue;
      return newValue;
    } else {
      velocityBelowThresholdTimer.stop();
      velocityBelowThresholdTimer.reset();
      lastValue = false;
      return false;
    }
  }

  @Override
  public boolean getAsBoolean() {
    return lastValue;
  }

  public Trigger asTrigger() {
    if (cachedTrigger == null) {
      cachedTrigger = new Trigger(this);
    }
    return cachedTrigger;
  }
}
