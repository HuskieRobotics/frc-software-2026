package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterConfiguration extends Command {

  private final SwerveDrivetrain drivetrain;
  private final Shooter shooter;

  private String gameData;

  public enum ShooterMode {
    SHOOT,
    SHOOT_OTM,
    COLLECT,
    NEAR_TRENCH,
    PASS
  }

  public ShooterConfiguration(SwerveDrivetrain drivetrain, Shooter shooter) {
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
      if (hubActive() && OISelector.getOperatorInterface().getShootOnTheMoveToggle().getAsBoolean()) {
        return ShooterMode.SHOOT_OTM;
      } else {
        return ShooterMode.SHOOT;
      }
    } else {
      if (OISelector.getOperatorInterface().getPassToggle().getAsBoolean()) { // pass toggeled by Operator
        return ShooterMode.PASS;
      } else {
        return ShooterMode.COLLECT;
      }
    }
  }

  public void getTrajectory() {
    if (getMode() == ShooterMode.NEAR_TRENCH) {
      // set hood to max
    } else if (getMode() == ShooterMode.SHOOT) {
      // model for aimed position
      // x stance
    } else if (getMode() == ShooterMode.COLLECT) {
      // model for aimed position
    } else if (getMode() == ShooterMode.SHOOT_OTM) {
      // model for OTM pos
    } else if (getMode() == ShooterMode.PASS) {
      // model for aimed position, which would be the nearest position
    } 
  }

  public void getTurret() {

    if (getMode() == ShooterMode.SHOOT || getMode() == ShooterMode.COLLECT) {
    } // aim turret at hub
    else if (getMode() == ShooterMode.SHOOT_OTM) {
    } // aim the turret at transforme dhub based on velocity
    else if (getMode() == ShooterMode.PASS) {
    } // aim turret at nearest corner
  }

  // based on match time (which should be equivalent to the timer of this command as it is enabled)
  // and game data to see which hub was active first
  // add a t second offset for how long it takes the ball (on average) to actually enter the hub
  public boolean hubActive() {
    return true;
  }
}
