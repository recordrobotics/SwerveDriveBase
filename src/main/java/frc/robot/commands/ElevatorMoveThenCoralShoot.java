package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorHeight;

public class ElevatorMoveThenCoralShoot extends SequentialCommandGroup {
  public ElevatorMoveThenCoralShoot(ElevatorHeight targetHeight) {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new ElevatorMove(targetHeight),
        new CoralShoot(),
        new ElevatorMove(ElevatorHeight.INTAKE)
    );
  }
}
