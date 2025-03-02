package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorAlgaeToggled extends Command {

  private ElevatorHeight targetHeight;

  public ElevatorAlgaeToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
    addRequirements(RobotContainer.elevatorMoveToggleRequirement);
  }

  @Override
  public void initialize() {
    new ElevatorMoveThenAlgaeGrab(targetHeight).handleInterrupt(this::cancel).schedule();
  }

  @Override
  public void end(boolean interrupted) {
    new ElevatorMoveThenAlgaeGrabEnd(targetHeight).handleInterrupt(this::cancel).schedule();
  }
}
