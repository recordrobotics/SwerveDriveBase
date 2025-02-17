package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight) {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.elevatorPattern),
        new ElevatorMove(targetHeight),
        new LightsCommand(LightSegments.ALGAE_GRABBER, Constants.Lights.PULSATING_ORANGE),
        new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.INTAKE)),
        new WaitUntilCommand(RobotContainer.algaeGrabber::hasAlgae),
        new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)),
        new LightsCommand(LightSegments.ALGAE_GRABBER, Constants.Lights.OFF),
        new SuccessfulCompletion(false, true, false, false, true),
        new ElevatorMove(ElevatorHeight.INTAKE),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.OFF),
        new SuccessfulCompletion(true, false, false, false, true));
  }
}
