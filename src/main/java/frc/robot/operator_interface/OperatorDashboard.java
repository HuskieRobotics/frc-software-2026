package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;
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

  public final LoggedTunableBoolean shootOnTheMoveToggle =
      new LoggedTunableBoolean("operatorDashboard/Shoot On The Move", true, true);

  public final LoggedTunableBoolean objectDetectionToggle =
      new LoggedTunableBoolean("operatorDashboard/Object Detection Enabled", true, true);

  public final LoggedTunableBoolean passToggle =
      new LoggedTunableBoolean("operatorDashboard/Pass", false, true);

  public final LoggedTunableBoolean shootOnTheMoveToggle =
      new LoggedTunableBoolean("operatorDashboard/Shoot On The Move", true, true);

  public final LoggedTunableBoolean hubActiveAtHomeToggle =
      new LoggedTunableBoolean("operatorDashboard/Hub Active (Home)", true, true);

  public OperatorDashboard() {}

  @Override
  public Trigger getPassToggle() {
    return new Trigger(() -> passToggle.get());
  }

  @Override
  public Trigger getShootOnTheMoveToggle() {
    return new Trigger(() -> shootOnTheMoveToggle.get());
  }

  @Override
  public Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> enableVision.get());
  }

  @Override
  public Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> enablePrimaryIRSensors.get());
  }

  @Override
  public Trigger getHubActiveAtHomeToggle() {
    return new Trigger(() -> (!DriverStation.isFMSAttached() && hubActiveAtHomeToggle.get()));
  }
  public Trigger getObjectDetectionToggle() {
    return new Trigger(() -> objectDetectionToggle.get());
  }
}
