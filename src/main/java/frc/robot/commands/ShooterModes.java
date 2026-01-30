package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterModes extends Command {

  private final SwerveDrivetrain drivetrain;
  private final Shooter shooter;

  // Shooter Mode Triggers
  private Trigger nearTrenchTrigger;
  private Trigger passModeTrigger;
  private Trigger collectAndHoldTrigger;
  private Trigger shootOTMTrigger;
  private Trigger canShootTrigger;

  private ShooterMode overrideMode = null;

  private String gameData;

  public enum ShooterMode {
    MANUAL_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trenh zone
    PASS // passing mode
  }

  public ShooterModes(SwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    configureShooterModeTriggers();
  }

  @Override
  public void initialize() {
    // Configure shooter settings here
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}

  private ShooterMode getMode() {

    if (overrideMode != null) {
      return overrideMode;
    }

    if (nearTrenchTrigger.getAsBoolean()) {
      return ShooterMode.NEAR_TRENCH;
    }

    else if (Field2d.getInstance().inAllianceZone()) {
      if (isHubActive()
          && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
        return ShooterMode.SHOOT_OTM;
      } else {
        return ShooterMode.MANUAL_SHOOT;
      }
    } else {
      if (OISelector.getOperatorInterface()
          .getPassToggle()
          .getAsBoolean()) { // pass toggeled by perator
        return ShooterMode.PASS;
      } else {
        return ShooterMode.COLLECT_AND_HOLD;
      }
    }
  }

  public boolean isShootOnTheMoveEnabled() {
    return getMode() == ShooterMode.SHOOT_OTM;
  }

  public boolean isCollectAndHoldEnabled() {
    return getMode() == ShooterMode.COLLECT_AND_HOLD;
  }

  public boolean isNearTrenchEnabled() {
    return getMode() == ShooterMode.NEAR_TRENCH;
  }

  public boolean isPassEnabled() {
    return getMode() == ShooterMode.PASS;
  }

  public boolean manualShootEnabled() {
    return getMode() == ShooterMode.MANUAL_SHOOT;
  }

  public void getTrajectory() {
    if (getMode() == ShooterMode.NEAR_TRENCH) {
      shooter.setIdleVelocity(); // FIXME: change to max hood angle
      // set hood to max
    } else if (getMode() == ShooterMode.MANUAL_SHOOT) {
      // model for aimed position
      // x stance
    } else if (getMode() == ShooterMode.COLLECT_AND_HOLD) {
      // model for aimed position
    } else if (getMode() == ShooterMode.SHOOT_OTM) {
      // model for OTM pos
    } else if (getMode() == ShooterMode.PASS) {
      // model for aimed position, which would be the nearest position
    }
  }

  public void getTurret() {

    if (getMode() == ShooterMode.MANUAL_SHOOT || getMode() == ShooterMode.COLLECT_AND_HOLD) {
    } // aim turret at hub
    else if (getMode() == ShooterMode.SHOOT_OTM) {
    } // aim the turret at transformed hub based on velocity
    else if (getMode() == ShooterMode.PASS) {
    } // aim turret at nearest corner
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub
  public boolean isHubActive() {
    return true;
  }

  private void setShooterMode(ShooterMode newMode) {
    this.overrideMode = newMode;
  }

  private void configureShooterModeTriggers() {
    nearTrenchTrigger = new Trigger(() -> Field2d.getInstance().inTrenchZone());
    nearTrenchTrigger.whileTrue(
        Commands.startEnd(
            () -> setShooterMode(ShooterMode.NEAR_TRENCH),
            () -> setShooterMode(null) // set it back so that it returns to automatic modes?
            ));

    passModeTrigger = new Trigger(
        () -> OISelector.getOperatorInterface().getPassToggle().getAsBoolean() && !Field2d.getInstance().inAllianceZone());
    
    passModeTrigger.onTrue(
        Commands.runOnce(() -> setShooterMode(ShooterMode.PASS)));   

    collectAndHoldTrigger = new Trigger(
        () -> !OISelector.getOperatorInterface().getPassToggle().getAsBoolean() && !Field2d.getInstance().inAllianceZone());

    collectAndHoldTrigger.onTrue(
        Commands.runOnce(() -> setShooterMode(ShooterMode.COLLECT_AND_HOLD)));

    shootOTMTrigger = new Trigger(
        () -> Field2d.getInstance().inAllianceZone()
            && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean() && !isNearTrenchEnabled() && isHubActive());

    shootOTMTrigger.onTrue(
        Commands.runOnce(() -> setShooterMode(ShooterMode.SHOOT_OTM)));

    canShootTrigger = new Trigger(
        () -> !OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean() && Field2d.getInstance().inAllianceZone() && !isNearTrenchEnabled() && isHubActive());

    canShootTrigger.onTrue(
        Commands.runOnce(() -> setShooterMode(ShooterMode.MANUAL_SHOOT)));
  }  
}
