// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI extends OperatorDashboard {
  private final CommandJoystick translateJoystick;
  private final Trigger[] translateJoystickButtons;

  private final CommandJoystick rotateJoystick;
  private final Trigger[] rotateJoystickButtons;

  private final CommandJoystick dilateJoystick;
  private final Trigger[] dilateJoystickButtons;

  private final CommandXboxController operatorController;

  public FullOperatorConsoleOI(int translatePort, int rotatePort, int operatorControllerPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    int dilatePort = 5;
    dilateJoystick = new CommandJoystick(dilatePort);
    operatorController = new CommandXboxController(operatorControllerPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.dilateJoystickButtons = new Trigger[13];
    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
      dilateJoystickButtons[i] = dilateJoystick.button(i);
    }
  }

  // Translation Joystick
  // binds different joystick buttons to different triggers, and then those triggers can be used in
  // the RobotContainer to bind commands to them, this allows for more flexibility in changing
  // button bindings without having to change code in the RobotContainer

  /**
   * // A positive rotation around the X axis moves the joystick right, and a // positive rotation
   * around the Y axis moves the joystick backward. When // treating them as translations, 0 radians
   * is measured from the right // direction, and angle increases clockwise. // // It's rotated 90
   * degrees CCW (y is negated and the arguments are reversed) // so that 0 radians is forward.
   */
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger getForceSafeShootButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getForceDeployIntakeButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getDeployRetractIntakeButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getSnakeDriveButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getInterruptAll() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[9];
  }

  @Override
  public Trigger getZeroHoodButton() {
    return translateJoystickButtons[10];
  }

  // Rotation Joystick
  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getManualShootButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getStartStopIntakeRollersButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getLimitAccelerationAndVelocityButton() {
    return operatorController.x();
  }

  @Override
  public Trigger getUnjamHopperButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getCurrentPoseButton() {
    return rotateJoystickButtons[6];
  }

  // Dilation joysticks
  // Since there is no actual dilation joystick, there are no methods or triggers to register with
  // it

  // Operator Controller
  @Override
  public Trigger getReverseIntakeRollersButton() {
    return operatorController.a();
  }

  @Override
  public Trigger getIncrementFlywheelVelocityButton() {
    return operatorController.povUp();
  }

  @Override
  public Trigger getIncrementRollerVelocityButton() {
    return operatorController.rightTrigger();
  }

  @Override
  public Trigger getDecrementRollerVelocityButton() {
    return operatorController.leftTrigger();
  }

  @Override
  public Trigger getDecrementFlywheelVelocityButton() {
    return operatorController.povDown();
  }

  @Override
  public Trigger getMoveTurretLeftButton() {
    return operatorController.povLeft();
  }

  @Override
  public Trigger getMoveTurretRightButton() {
    return operatorController.povRight();
  }

  @Override
  public Trigger getDrivetrainSystemTest() {
    return operatorController.back().and(operatorController.b());
  }

  @Override
  public Trigger getShooterSystemTest() {
    return operatorController.back().and(operatorController.start());
  }

  @Override
  public Trigger getIntakeSystemTest() {
    return operatorController.back().and(operatorController.y());
  }

  @Override
  public Trigger getHopperSystemTest() {
    return operatorController.back().and(operatorController.x());
  }

  @Override
  public Trigger getClearAllFaults() {
    return operatorController.leftBumper().and(operatorController.rightBumper());
  }

  @Override
  public Trigger getCheckForFaults() {
    return operatorController.rightBumper();
  }
}
