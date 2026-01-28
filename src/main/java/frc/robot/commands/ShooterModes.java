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

  private String gameData;

  public enum ShooterMode {
    CAN_SHOOT, // Shoot only when we want
    SHOOT_OTM, // shoot on the move
    COLLECT_AND_HOLD, // collecting and holding fuel in hopper
    NEAR_TRENCH, // near the trenh zone
    PASS // passing mode
  }

  public ShooterModes(SwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
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

  public ShooterMode getMode() {
    if (Field2d.getInstance().inTrenchZone()) {
      return ShooterMode.NEAR_TRENCH;
    }

    if (Field2d.getInstance().inAllianceZone()) {
      if (hubActive()
          && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
        return ShooterMode.SHOOT_OTM;
      } else {
        return ShooterMode.CAN_SHOOT;
      }
    } else {
      if (OISelector.getOperatorInterface()
          .getPassToggle()
          .getAsBoolean()) { // pass toggeled by Operator
        return ShooterMode.PASS;
      } else {
        return ShooterMode.COLLECT_AND_HOLD;
      }
    }
  }

  public void getTrajectory() {
    if (getMode() == ShooterMode.NEAR_TRENCH) {
      shooter.setIdleVelocity(); // FIXME: change to max hood angle
      // set hood to max
    } else if (getMode() == ShooterMode.CAN_SHOOT) {
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

    if (getMode() == ShooterMode.CAN_SHOOT || getMode() == ShooterMode.COLLECT_AND_HOLD) {
    } // aim turret at hub
    else if (getMode() == ShooterMode.SHOOT_OTM) {
    } // aim the turret at transformed hub based on velocity
    else if (getMode() == ShooterMode.PASS) {
    } // aim turret at nearest corner
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub
  public boolean hubActive() {
    return true;
  }

  private void configureShooterModeTriggers() {
    nearTrenchTrigger = new Trigger(() -> Field2d.getInstance().inTrenchZone());
    nearTrenchTrigger.onTrue(Commands.none());
  }
}
