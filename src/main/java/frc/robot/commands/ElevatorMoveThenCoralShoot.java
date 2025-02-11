package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorMoveThenCoralShoot extends SequentialCommandGroup {
  public ElevatorMoveThenCoralShoot(ElevatorHeight targetHeight) {
    addCommands(
        new ConditionalCommand(
            new InstantCommand(() -> this.cancel()),
            new InstantCommand(() -> {}),
            () -> !RobotContainer.coralShooter.hasCoral()),
        new ElevatorMove(targetHeight),
        new CoralShoot(),
        new InstantCommand(() -> new ElevatorMove(ElevatorHeight.BOTTOM).schedule()));
  }
}
