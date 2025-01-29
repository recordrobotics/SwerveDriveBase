package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorAlgae.ElevatorAlgaeStates;

public class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight) {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new ElevatorMove(targetHeight),
        new InstantCommand(() -> RobotContainer.elevatorAlgae.toggle(ElevatorAlgaeStates.INTAKE)),
        new WaitUntilCommand(RobotContainer.elevatorAlgae::hasAlgae),
        new InstantCommand(() -> RobotContainer.elevatorAlgae.toggle(ElevatorAlgaeStates.OFF)),
        new ElevatorMove(ElevatorHeight.INTAKE));
  }
}
