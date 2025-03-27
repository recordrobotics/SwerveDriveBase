package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromGroundSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;

public class CoralIntakeFromGround extends SequentialCommandGroup {
  public CoralIntakeFromGround() {
    addRequirements(RobotContainer.coralIntake);
    addRequirements(RobotContainer.elevatorHead);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.coralIntakePattern)
                .onlyWhile(this::isScheduled)),
        new InstantCommand(
            () -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.DOWN),
            RobotContainer.coralIntake),
        // start moving elevator to intake position
        new ScheduleCommand(new ElevatorMove(ElevatorHeight.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_GREEN)
                .onlyWhile(this::isScheduled)),
        // start intaking
        new InstantCommand(
            () -> RobotContainer.coralIntake.toggle(CoralIntakeStates.INTAKE),
            RobotContainer.coralIntake),
        new ScheduleCommand(new CoralIntakeFromGroundSim(0.2))
            .onlyIf(() -> Constants.RobotState.getMode() != Mode.REAL));
  }
}
