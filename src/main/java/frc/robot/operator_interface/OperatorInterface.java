// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  // DRIVER TRIGGERS, mostly game-specific

  public default Trigger getForceSafeShootButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLimitAccelerationAndVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getManualShootButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getStartStopIntakeRollersButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getUnjamHopperButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getSnakeDriveButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCurrentPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getReverseIntakeRollersButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  public default Trigger getDeployRetractIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getForceDeployIntakeButton() {
    return new Trigger(() -> false);
  }

  // OPERATOR TRIGGERS, mostly game-specific
  public default Trigger getPassToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getLockTurretForBankToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootOnTheMoveToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getPassOnTheMoveToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getObjectDetectionToggle() {
    return new Trigger(() -> false);
  }

  public default Trigger getLockShooterToggle() {
    return new Trigger(() -> false);
  }

  // Operator Xbox Controller Triggers
  public default Trigger getIncrementFlywheelVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDecrementFlywheelVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIncrementRollerVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDecrementRollerVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveTurretRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveTurretLeftButton() {
    return new Trigger(() -> false);
  }

  // only for at home
  public default Trigger getHubActiveAtHomeToggle() {
    return new Trigger(() -> false);
  }

  // Shooter triggers
  public default Trigger getZeroHoodButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoSnapsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getSlowShooterForPitTest() {
    return new Trigger(() -> false);
  }

  // System Tests

  public default Trigger getDrivetrainSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getShooterSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getHopperSystemTest() {
    return new Trigger(() -> false);
  }

  // Faults

  public default Trigger getClearAllFaults() {
    return new Trigger(() -> false);
  }

  public default Trigger getCheckForFaults() {
    return new Trigger(() -> false);
  }
}
