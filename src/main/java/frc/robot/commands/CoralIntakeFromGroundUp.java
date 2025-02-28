package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromGroundUp extends SequentialCommandGroup {
  public CoralIntakeFromGroundUp() {
    addRequirements(RobotContainer.coralIntake);
    addRequirements(RobotContainer.coralShooter);
    addRequirements(RobotContainer.elevator);

    addCommands(
        // raise the arm
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        // once both the arm and elevator are at goal, start elevator intake
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE)),
        // push coral out
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE)),
        // wait for elevator to have coral
        new CoralIntakeToElevator()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP)),
        // move coral a set distance
        new InstantCommand(
            () -> RobotContainer.coralShooter.moveBy(Constants.CoralShooter.CORAL_INTAKE_DISTANCE)),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.positionAtGoal()),
        // stop elevator intake
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)),
        // stop intake push out
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)));
  }
}
