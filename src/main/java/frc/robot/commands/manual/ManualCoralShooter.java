package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class ManualCoralShooter extends Command {
  private CoralShooterStates targetState;

  public ManualCoralShooter(CoralShooterStates targetState) {
    this.targetState = targetState;
    addRequirements(RobotContainer.coralShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.coralShooter.toggle(targetState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.coralShooter.kill();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
