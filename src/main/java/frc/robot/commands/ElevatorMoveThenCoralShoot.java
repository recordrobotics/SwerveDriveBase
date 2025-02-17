package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;

public class ElevatorMoveThenCoralShoot extends SequentialCommandGroup {
  public ElevatorMoveThenCoralShoot(ElevatorHeight targetHeight) {
    addCommands(
        new ConditionalCommand(
            new InstantCommand(() -> this.cancel()),
            new InstantCommand(() -> {}),
            () -> !RobotContainer.coralShooter.hasCoral()),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.elevatorPattern),
        new ElevatorMove(targetHeight),
        new CoralShoot(),
        new InstantCommand(
            () ->
                new ElevatorMove(ElevatorHeight.BOTTOM)
                    .andThen(new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.OFF))
                    .schedule()));
  }
}
