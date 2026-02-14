package frc.robot.visualizations;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.RobotConfig;
import frc.robot.subsystems.climber.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualization {

  // Elevator: White

  private Climber climber;

  private LoggedMechanism2d climberVisualization2D;

  private LoggedMechanismLigament2d climberBox;
  private final double kClimberRootPosY = Units.inchesToMeters(4.875);
  private final double kClimberRootPosZ = Units.inchesToMeters(20.25);
  private final double kClimberLength = Units.inchesToMeters(10.0);

  public RobotVisualization(Climber climber) {
    this.climber = climber;
    if (RobotBase.isReal()) return;
    init2dVisualization();
  }

  private void init2dVisualization() {
    climberVisualization2D =
        new LoggedMechanism2d(
            RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters), 3.0);

    // Climber
    LoggedMechanismRoot2d climberRoot =
        climberVisualization2D.getRoot("climberRoot", kClimberRootPosY, kClimberRootPosZ);
    climberBox =
        new LoggedMechanismLigament2d(
            "climber", kClimberLength, 0.0, 5.0, new Color8Bit(Color.kOrange));
    climberRoot.append(this.climberBox);
  }

  public void update() {
    if (RobotBase.isReal()) return;

    climberBox.setAngle(climber.getAngle().in(Degrees));

    Logger.recordOutput("Visualization/Climber", this.climberVisualization2D);
  }
}
