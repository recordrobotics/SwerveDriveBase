package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ProcessorScore extends SequentialCommandGroup {

  /** NOTE! This command HAS to be deferred */
  public ProcessorScore() {
    addRequirements(RobotContainer.algaeGrabber);

    addCommands(
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .stateVisualizer
                    .runPattern(Constants.Lights.algaeScorePattern)
                    .schedule()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .algaeGrabber
                    .runPattern(Constants.Lights.PULSATING_ORANGE)
                    .schedule()));

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
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .algaeGrabber
                    .runPattern(Constants.Lights.FLASHING_GREEN)
                    .alongWith(
                        RobotContainer.lights.stateVisualizer.runPattern(
                            Constants.Lights.PULSATING_GREEN))
                    .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)
                    .schedule()));
  }
}
