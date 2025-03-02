package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorReefToggled extends Command {

  private ElevatorHeight targetHeight;
  private Command lightsCommand;
  private Command elevatorCommand;

  public ElevatorReefToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
    addRequirements(RobotContainer.elevator);
  }

  @Override
  public void initialize() {
    lightsCommand = RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);
    lightsCommand.schedule();

    elevatorCommand = new ElevatorMove(targetHeight);
    elevatorCommand.initialize();
  }

  @Override
  public void execute() {
    if (!elevatorCommand.isFinished()) {
      elevatorCommand.execute();

      if (elevatorCommand.isFinished()) {
        elevatorCommand.end(false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!elevatorCommand.isFinished()) elevatorCommand.end(interrupted);

    new InstantCommand(
            () ->
                new ElevatorMove(ElevatorHeight.BOTTOM)
                    .andThen(new InstantCommand(lightsCommand::cancel))
                    .schedule())
        .schedule();
  }
}
