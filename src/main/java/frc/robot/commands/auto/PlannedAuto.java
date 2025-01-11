package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PlannedAuto extends SequentialCommandGroup {
  public PlannedAuto() {
    addCommands(
        ShuffleboardUI.Autonomous.getAutoChooser(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
  }
}
