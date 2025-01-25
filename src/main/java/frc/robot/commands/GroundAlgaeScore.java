package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GroundAlgae.GroundAlgaeStates;

public class GroundAlgaeScore extends SequentialCommandGroup {
  public GroundAlgaeScore() {
    addRequirements(RobotContainer.groundAlgae);

    addCommands(
        new InstantCommand(() -> RobotContainer.groundAlgae.toggle(GroundAlgaeStates.OUT)),
        new WaitUntilCommand(() -> RobotContainer.groundAlgae.hasAlgae()),
        new InstantCommand(() -> RobotContainer.groundAlgae.toggle(GroundAlgaeStates.OFF)));
  }
}
