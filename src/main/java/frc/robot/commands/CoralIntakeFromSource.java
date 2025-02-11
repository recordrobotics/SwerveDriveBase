package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromSourceSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromSource extends SequentialCommandGroup {
  public CoralIntakeFromSource() {
    addCommands(
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP)),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)),
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new CoralIntakeFromSourceSim()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)));
  }
}
