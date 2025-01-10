package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.Drivetrain;

public class PlannedAuto extends SequentialCommandGroup {
  public PlannedAuto(Drivetrain drivetrain) {
    addCommands(
        ShuffleboardUI.Autonomous.getAutoChooser(), new InstantCommand(() -> drivetrain.kill()));
  }
}
