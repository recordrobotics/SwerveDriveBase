package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevator extends Command {

  public ManualElevator() {
    addRequirements(RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    RobotContainer.elevator.toggle(
        RobotContainer.elevator.getCurrentHeight() + controls.getManualElevator());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
