package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromSourceSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromSource extends SequentialCommandGroup {
  public CoralIntakeFromSource() {
    addCommands(
        // new ConditionalCommand(
        //     new InstantCommand(() -> this.cancel()),
        //     new InstantCommand(() -> {}),
        //     () -> RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .elevator
                    .runPattern(Constants.Lights.elevatorPattern)
                    .schedule()),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .coralIntake
                    .runPattern(Constants.Lights.coralIntakePattern)
                    .schedule()),
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
                    .runPattern(Constants.Lights.PULSATING_ORANGE)
                    .schedule()),
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.INTAKE)),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE)),
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new CoralIntakeFromSourceSim()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        // move coral a set distance
        new InstantCommand(
            () -> RobotContainer.coralShooter.moveBy(Constants.CoralShooter.CORAL_INTAKE_DISTANCE)),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.positionAtGoal()),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)),
        new InstantCommand(
            () ->
                RobotContainer.lights
                    .elevator
                    .runPattern(Constants.Lights.FLASHING_GREEN)
                    .alongWith(
                        RobotContainer.lights.coralIntake.runPattern(
                            Constants.Lights.FLASHING_GREEN))
                    .alongWith(
                        RobotContainer.lights.coralShooter.runPattern(
                            Constants.Lights.FLASHING_GREEN))
                    .alongWith(
                        RobotContainer.lights.stateVisualizer.runPattern(
                            Constants.Lights.PULSATING_GREEN))
                    .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)
                    .schedule()));
  }
}
