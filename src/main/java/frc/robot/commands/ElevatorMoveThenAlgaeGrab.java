package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight) {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new ElevatorMove(targetHeight),
        new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.INTAKE)),
        new WaitUntilCommand(RobotContainer.algaeGrabber::hasAlgae),
        new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)),
        new ElevatorMove(ElevatorHeight.INTAKE));
  }
}
