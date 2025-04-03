package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CoralIntakeFromGroundToggled extends Command {

  public static boolean isGoingToL1 = false;

  public CoralIntakeFromGroundToggled() {
    addRequirements(RobotContainer.coralIntakeMoveToggleRequirement);
  }

  @Override
  public void initialize() {
    new CoralIntakeFromGround().handleInterrupt(this::cancel).schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (!isGoingToL1) {
      new CoralIntakeFromGroundUp().handleInterrupt(this::cancel).schedule();
    }
  }
}
