package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorMove extends Command {
  private ElevatorHeight targetHeight;

  public ElevatorMove(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
    addRequirements(RobotContainer.elevator, RobotContainer.elevatorArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevator.moveTo(targetHeight);
    RobotContainer.elevatorArm.toggle(targetHeight.getArmAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.elevator.moveTo(targetHeight);
    RobotContainer.elevatorArm.toggle(targetHeight.getArmAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevator.atGoal() && RobotContainer.elevatorArm.atGoal();
  }
}
