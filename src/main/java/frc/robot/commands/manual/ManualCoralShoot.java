package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class ManualCoralShoot extends SequentialCommandGroup {
  public ManualCoralShoot(CoralShooterStates targetState) {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(targetState)),
        new WaitCommand(Constants.CoralShooter.SHOOT_TIME),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)));
  }
}
