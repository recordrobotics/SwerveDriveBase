package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import java.util.Set;

public class ElevatorMoveThenAlgaeGrabEnd extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight targetHeight) {
    addRequirements(RobotContainer.algaeGrabber);

    addCommands(
        new DeferredCommand(
            () ->
                new ElevatorMove(
                    targetHeight == ElevatorHeight.GROUND_ALGAE
                        ? ElevatorHeight.GROUND_ALGAE
                        : ElevatorHeight.INTAKE),
            Set.of(RobotContainer.elevator)),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .elevator
                    .runPattern(Constants.Lights.FLASHING_GREEN)
                    .alongWith(
                        RobotContainer.lights.stateVisualizer.runPattern(
                            Constants.Lights.PULSATING_GREEN))
                    .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)
                    .schedule()));
  }
}
