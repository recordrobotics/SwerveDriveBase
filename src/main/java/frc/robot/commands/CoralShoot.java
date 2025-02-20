package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralShooterToReef;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralShoot extends SequentialCommandGroup {
  public CoralShoot() {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .coralShooter
                    .runPattern(Constants.Lights.coralShooterPattern)
                    .schedule()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .stateVisualizer
                    .runPattern(Constants.Lights.coralScorePattern)
                    .schedule()),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OUT)),
        // Make sure coral left
        new CoralShooterToReef()
            .simulateFor(new WaitUntilCommand(() -> !RobotContainer.coralShooter.hasCoral())),
        new WaitCommand(Constants.CoralShooter.SHOOT_TIME),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .coralShooter
                    .runPattern(Constants.Lights.FLASHING_GREEN)
                    .alongWith(
                        RobotContainer.lights.stateVisualizer.runPattern(
                            Constants.Lights.PULSATING_GREEN))
                    .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)
                    .schedule()));
  }
}
