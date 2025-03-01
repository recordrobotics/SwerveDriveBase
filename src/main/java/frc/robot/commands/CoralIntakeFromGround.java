package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;

public class CoralIntakeFromGround extends SequentialCommandGroup {
  public CoralIntakeFromGround() {
    addRequirements(RobotContainer.coralIntake);
    addRequirements(RobotContainer.coralShooter);
    addRequirements(RobotContainer.elevator);

    addCommands(
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .stateVisualizer
                    .runPattern(Constants.Lights.PULSATING_ORANGE)
                    .schedule()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .coralIntake
                    .runPattern(Constants.Lights.coralIntakePattern)
                    .schedule()),
        // start moving elevator to intake position
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        // at the same time lower the arm
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.DOWN)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .coralIntake
                    .runPattern(Constants.Lights.PULSATING_GREEN)
                    .schedule()),
        // start intaking
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.INTAKE)));
  }
}
