package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorReefToggled extends Command {

  private ElevatorHeight targetHeight;
  private Command lightsCommand;

  public ElevatorReefToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
    addRequirements(RobotContainer.elevatorMoveToggleRequirement);
  }

  @Override
  public void initialize() {
    lightsCommand = RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);
    lightsCommand.schedule();

    new InstantCommand(
            () -> new ElevatorMove(targetHeight).handleInterrupt(this::cancel).schedule())
        .schedule();
  }

  @Override
  public void end(boolean interrupted) {
    new InstantCommand(
            () ->
                new ElevatorMove(ElevatorHeight.BOTTOM)
                    .handleInterrupt(this::cancel)
                    .andThen(new InstantCommand(lightsCommand::cancel))
                    .schedule())
        .schedule();
  }
}
