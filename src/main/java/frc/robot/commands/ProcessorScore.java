package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ProcessorScore extends SequentialCommandGroup {
  public ProcessorScore() {
    addRequirements(RobotContainer.algaeGrabber);
    addCommands(
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.algaeScorePattern),
        new LightsCommand(LightSegments.ALGAE_GRABBER, Constants.Lights.PULSATING_ORANGE));
    if (RobotContainer.elevator.getHeight() == ElevatorHeight.GROUND_ALGAE) {
      addCommands(
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.INTAKE)),
          new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)));
    } else {
      addCommands(
          new ElevatorMove(ElevatorHeight.PROCESSOR_SCORE),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OUT)),
          new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)));
    }
    addCommands(
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.OFF),
        new LightsCommand(LightSegments.ALGAE_GRABBER, Constants.Lights.OFF),
        new SuccessfulCompletion(false, true, false, false, true));
  }
}
