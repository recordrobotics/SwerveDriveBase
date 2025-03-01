package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberSim;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight) {
    Command algaeGrabberLightsCommand =
        RobotContainer.lights.algaeGrabber.runPattern(Constants.Lights.PULSATING_ORANGE);

    addRequirements(RobotContainer.algaeGrabber);

    addCommands(
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .elevator
                    .runPattern(Constants.Lights.elevatorPattern)
                    .schedule()),
        new ElevatorMove(targetHeight),
        new InstantCommand(() -> algaeGrabberLightsCommand.schedule()),
        new InstantCommand(
            () -> {
              if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.INTAKE_GROUND);
              else RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.INTAKE_REEF);
            }),
        new AlgaeGrabberSim(0.2)
            .simulateFor(new WaitUntilCommand(RobotContainer.algaeGrabber::hasAlgae)),
        new InstantCommand(() -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF)),
        new InstantCommand(algaeGrabberLightsCommand::cancel),
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
