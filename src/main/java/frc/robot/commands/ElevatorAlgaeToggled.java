package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;

public class ElevatorAlgaeToggled extends Command {

  private ElevatorHeight targetHeight;

  public ElevatorAlgaeToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
  }

  @Override
  public void initialize() {
    new ElevatorMoveThenAlgaeGrab(targetHeight).schedule();
  }

  @Override
  public void end(boolean interrupted) {
    new ElevatorMoveThenAlgaeGrabEnd(targetHeight).schedule();
  }
}
