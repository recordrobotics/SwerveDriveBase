package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromGroundTimeBased;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromGround extends SequentialCommandGroup {
  public CoralIntakeFromGround() {
    addRequirements(RobotContainer.coralIntake);
    addRequirements(RobotContainer.coralShooter);
    addRequirements(RobotContainer.elevator);

    addCommands(
        // start moving elevator to intake position
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        // at the same time lower the arm
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.DOWN)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        // start intaking
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.INTAKE)),
        // wait until coral is detected
        new CoralIntakeFromGroundTimeBased(0.2)
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralIntake.hasCoral())),
        // raise the arm
        new InstantCommand(() -> RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        // once both the arm and elevator are at goal, start elevator intake
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE)),
        // push coral out
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE)),
        // wait for elevator to have coral
        new CoralIntakeToElevator()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        // stop elevator intake
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF)),
        // stop intake push out
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)));
  }
}
