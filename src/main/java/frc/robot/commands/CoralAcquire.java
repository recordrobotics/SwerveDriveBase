package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralAquisition.CoralAquisitionStates;

public class CoralAcquire extends SequentialCommandGroup {
  public CoralAcquire(CoralAquisitionStates targetState) {
    addRequirements(RobotContainer.coralAquisition);

    addCommands(
        new InstantCommand(() -> RobotContainer.coralAquisition.toggle(targetState)),
        new WaitCommand(Constants.CoralAquisition.ACQUIRE_TIME),
        new InstantCommand(() -> RobotContainer.coralAquisition.toggle(CoralAquisitionStates.OFF)));
  }
}
