package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;

public class CoralIntakeFromGroundUp extends SequentialCommandGroup {
  public CoralIntakeFromGroundUp(boolean moveToElevator) {
    addRequirements(RobotContainer.coralIntake);

    if (moveToElevator) {
      addRequirements(RobotContainer.elevatorHead);
      addRequirements(RobotContainer.elevator);
    }

    addCommands(
        // raise the arm
        new InstantCommand(
            () -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP),
            RobotContainer.coralIntake),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> !moveToElevator || RobotContainer.elevator.atGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        Commands.either(
            // once both the arm and elevator are at goal, start elevator intake
            new InstantCommand(
                    () -> RobotContainer.elevatorHead.toggle(CoralShooterStates.INTAKE),
                    RobotContainer.elevatorHead)
                .andThen(
                    // push coral out
                    new InstantCommand(
                        () -> RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE),
                        RobotContainer.coralIntake))
                .andThen(
                    // wait for elevator to have coral
                    new CoralIntakeToElevator()
                        .simulateFor(
                            new WaitUntilCommand(() -> RobotContainer.elevatorHead.hasCoral())))
                .andThen(
                    new InstantCommand(
                        () -> {
                          RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP);
                          RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF);
                        },
                        RobotContainer.coralIntake))
                .andThen(
                    // move coral a set distance
                    new InstantCommand(
                        () ->
                            RobotContainer.elevatorHead.moveBy(
                                Constants.ElevatorHead.CORAL_INTAKE_DISTANCE),
                        RobotContainer.elevatorHead))
                .andThen(
                    new WaitUntilCommand(() -> RobotContainer.elevatorHead.positionAtGoal()),
                    // stop elevator intake
                    new InstantCommand(
                        () -> RobotContainer.elevatorHead.toggle(CoralShooterStates.OFF),
                        RobotContainer.elevatorHead)),
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF);
                },
                RobotContainer.coralIntake),
            () -> moveToElevator),
        new ScheduleCommand(
            RobotContainer.lights
                .elevator
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.coralIntake.runPattern(Constants.Lights.FLASHING_GREEN))
                .alongWith(
                    RobotContainer.lights.coralShooter.runPattern(Constants.Lights.FLASHING_GREEN))
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }
}
