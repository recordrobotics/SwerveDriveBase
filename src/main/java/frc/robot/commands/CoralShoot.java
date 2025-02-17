package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralShooterToReef;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralShoot extends SequentialCommandGroup {
  public CoralShoot() {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new LightsCommand(LightSegments.CORAL_SHOOTER, Constants.Lights.coralShooterPattern),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.coralScorePattern),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OUT)),
        // Make sure coral left
        new CoralShooterToReef()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        new WaitCommand(Constants.CoralShooter.SHOOT_TIME),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.OFF),
        new LightsCommand(LightSegments.CORAL_SHOOTER, Constants.Lights.OFF),
        new SuccessfulCompletion(false, false, false, true, true));
  }
}
