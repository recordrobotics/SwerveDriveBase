package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeFromGroundToggled extends Command {

  @Override
  public void initialize() {
    new CoralIntakeFromGround().schedule();
  }

  @Override
  public void end(boolean interrupted) {
    new CoralIntakeFromGroundUp().schedule();
  }
}
