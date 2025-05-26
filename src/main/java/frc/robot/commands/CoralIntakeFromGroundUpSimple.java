package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;

public class CoralIntakeFromGroundUpSimple extends SequentialCommandGroup {
  public CoralIntakeFromGroundUpSimple() {
    addRequirements(RobotContainer.coralIntake);

    addRequirements(RobotContainer.elevatorHead);
    addRequirements(RobotContainer.elevator);

    addCommands(
        // raise the arm
        new InstantCommand(
            () -> RobotContainer.coralIntake.set(CoralIntakeState.PUSH_READY),
            RobotContainer.coralIntake),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        // once both the arm and elevator are at goal, start elevator intake
        new InstantCommand(
            () -> RobotContainer.elevatorHead.set(CoralShooterStates.INTAKE),
            RobotContainer.elevatorHead),
        // push coral out
        new InstantCommand(
            () -> RobotContainer.coralIntake.set(CoralIntakeState.PUSH_OUT),
            RobotContainer.coralIntake),
        new WaitCommand(0.1));
  }
}
