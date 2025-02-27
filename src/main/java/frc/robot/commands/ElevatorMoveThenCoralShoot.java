package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorMoveThenCoralShoot extends SequentialCommandGroup {

  public ElevatorMoveThenCoralShoot(ElevatorHeight targetHeight) {
    Command lightsCommand =
        RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);

    addCommands(
        // new ConditionalCommand(
        //     new InstantCommand(() -> this.cancel()),
        //     new InstantCommand(() -> {}),
        //     () -> !RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(() -> lightsCommand.schedule()),
        new ElevatorMove(targetHeight).asProxy(),
        new CoralShoot(),
        new InstantCommand(
            () ->
                new ElevatorMove(ElevatorHeight.BOTTOM)
                    .asProxy()
                    .andThen(new InstantCommand(lightsCommand::cancel))
                    .schedule()));
  }
}
