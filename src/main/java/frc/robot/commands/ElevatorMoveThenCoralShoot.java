package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorHeight;

public class ElevatorMoveThenCoralShoot extends SequentialCommandGroup {
  public ElevatorMoveThenCoralShoot(ElevatorHeight targetHeight) {
    addCommands(
        new ElevatorMove(targetHeight),
        new CoralShoot(),
        new InstantCommand(() -> new ElevatorMove(ElevatorHeight.BOTTOM).schedule()));
  }
}
