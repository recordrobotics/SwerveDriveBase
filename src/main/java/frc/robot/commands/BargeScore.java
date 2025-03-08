package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberToProcessor;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class BargeScore extends SequentialCommandGroup {

  public BargeScore() {
    addRequirements(RobotContainer.algaeGrabber, RobotContainer.elevatorArm);

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
        new ElevatorMove(ElevatorHeight.BARGE_ALAGAE)
            .andThen(
                new InstantCommand(
                    () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.SHOOT_BARGE),
                    RobotContainer.algaeGrabber))
            .andThen(
                new AlgaeGrabberToProcessor()
                    .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME_BARGE)))
            .andThen(
                new InstantCommand(
                    () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF),
                    RobotContainer.algaeGrabber))
            .andThen(new ElevatorMove(ElevatorHeight.BOTTOM)),
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
