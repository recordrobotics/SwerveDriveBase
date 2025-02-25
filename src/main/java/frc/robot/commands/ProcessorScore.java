package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberToProcessor;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;
import java.util.Set;

public class ProcessorScore extends SequentialCommandGroup {

  public static DeferredCommand deferred() {
    return new DeferredCommand(
        () -> new ProcessorScore(), Set.of(RobotContainer.algaeGrabber, RobotContainer.elevator));
  }

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
          new AlgaeGrabberToProcessor()
              .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME)),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)),
          new ElevatorMove(ElevatorHeight.BOTTOM));
    } else {
      addCommands(
          new ElevatorMove(ElevatorHeight.PROCESSOR_SCORE),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OUT)),
          new AlgaeGrabberToProcessor()
              .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME)),
          new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)),
          new ElevatorMove(ElevatorHeight.BOTTOM));
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
