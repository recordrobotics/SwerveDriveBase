package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberToProcessor;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;

public class ProcessorScore extends SequentialCommandGroup {

  public ProcessorScore() {
    addRequirements(RobotContainer.elevatorHead);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.algaeScorePattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .algaeGrabber
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        Commands.either(
            new ElevatorMove(ElevatorHeight.GROUND_ALGAE_PROCESSOR)
                .asProxy()
                .andThen(
                    new InstantCommand(
                        () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OUT_GROUND),
                        RobotContainer.elevatorHead))
                .andThen(
                    new AlgaeGrabberToProcessor()
                        .simulateFor(new WaitCommand(Constants.ElevatorHead.SHOOT_TIME_GROUND)))
                .andThen(
                    new InstantCommand(
                        () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OFF),
                        RobotContainer.elevatorHead))
                .andThen(new ElevatorMove(ElevatorHeight.BOTTOM).asProxy()),
            Commands.either(
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.SHOOT_BARGE),
                        RobotContainer.elevatorHead)
                    .andThen(
                        new AlgaeGrabberToProcessor()
                            .simulateFor(new WaitCommand(Constants.ElevatorHead.SHOOT_TIME_BARGE)))
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OFF),
                            RobotContainer.elevatorHead)),
                new ElevatorMove(ElevatorHeight.PROCESSOR_SCORE)
                    .asProxy()
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OUT_REEF),
                            RobotContainer.elevatorHead))
                    .andThen(
                        new AlgaeGrabberToProcessor()
                            .simulateFor(new WaitCommand(Constants.ElevatorHead.SHOOT_TIME_REEF)))
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OFF),
                            RobotContainer.elevatorHead))
                    .andThen(new ElevatorMove(ElevatorHeight.BOTTOM).asProxy()),
                () -> RobotContainer.elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE),
            () -> RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE),
        new ScheduleCommand(
            RobotContainer.lights
                .algaeGrabber
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }
}
