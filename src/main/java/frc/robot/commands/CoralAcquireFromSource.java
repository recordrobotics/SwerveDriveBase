package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralAquisition.CoralAquisitionStates;

public class CoralAcquireFromSource extends SequentialCommandGroup {
  public CoralAcquireFromSource() {
    addRequirements(RobotContainer.coralAquisition);

    addCommands(
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new InstantCommand(
            () -> RobotContainer.coralAquisition.toggle(CoralAquisitionStates.ACQUIRE)),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(() -> RobotContainer.coralAquisition.toggle(CoralAquisitionStates.OFF)));
  }
}
