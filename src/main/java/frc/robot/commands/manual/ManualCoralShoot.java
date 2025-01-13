package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class ManualCoralShoot extends SequentialCommandGroup {
  public ManualCoralShoot(CoralShooterStates targetState) {
    addRequirements(RobotContainer.coralShooter);
    boolean previousHasCoral = RobotContainer.coralShooter.hasCoral();

    addCommands(
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(targetState)),
        new WaitCommand(Constants.CoralShooter.SHOOT_TIME),
        // Make sure coral status has changed
        new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral() != previousHasCoral),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)));
  }
}
