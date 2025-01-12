package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lights.LightMode;

public class LightSignal extends Command {
  private LightMode mode;
  private Supplier<Boolean> until;

  public LightSignal(LightMode mode, Supplier<Boolean> until) {
    this.mode = mode;
    this.until = until;
    addRequirements(RobotContainer.lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.lights.setMode(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return until.get();
  }
}