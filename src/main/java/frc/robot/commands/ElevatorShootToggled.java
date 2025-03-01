package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorShootToggled extends Command {

  private ElevatorHeight targetHeight;
  private Command lightsCommand;

  public ElevatorShootToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
  }

  @Override
  public void initialize() {
    lightsCommand = RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);

    new InstantCommand(() -> lightsCommand.schedule())
        .andThen(new ElevatorMove(targetHeight).asProxy());
  }

  @Override
  public void end(boolean interrupted) {
    new CoralShoot()
        .andThen(
            new InstantCommand(
                () ->
                    new ElevatorMove(ElevatorHeight.BOTTOM)
                        .asProxy()
                        .andThen(new InstantCommand(lightsCommand::cancel))
                        .schedule()))
        .schedule();
  }
}
