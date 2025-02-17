package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromSourceSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromSource extends SequentialCommandGroup {
  public CoralIntakeFromSource() {
    addCommands(
        new ConditionalCommand(
            new InstantCommand(() -> this.cancel()),
            new InstantCommand(() -> {}),
            () -> RobotContainer.coralShooter.hasCoral()),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.elevatorPattern),
        new LightsCommand(LightSegments.CORAL_INTAKE, Constants.Lights.coralIntakePattern),
        new LightsCommand(LightSegments.CORAL_SHOOTER, Constants.Lights.coralShooterPattern),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.PULSATING_ORANGE),
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP)),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)),
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
        new LightsCommand(LightSegments.CORAL_SHOOTER, Constants.Lights.OFF),
        new LightsCommand(LightSegments.CORAL_INTAKE, Constants.Lights.OFF),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.OFF),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.OFF),
        new SuccessfulCompletion(true, false, true, true, true));
  }
}
