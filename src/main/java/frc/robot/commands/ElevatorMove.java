package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class ElevatorMove extends Command {
    private ElevatorHeight targetHeight;
    private double lastMoveTime = 0;
    private double initialElevatorHeight;
    private double initialArmAngle;

    public ElevatorMove(ElevatorHeight targetHeight) {
        this.targetHeight = targetHeight;
        addRequirements(RobotContainer.elevator, RobotContainer.elevatorArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastMoveTime = Timer.getFPGATimestamp();

        initialElevatorHeight = RobotContainer.elevator.getCurrentHeight();
        initialArmAngle = RobotContainer.elevatorArm.getArmAngle();

        RobotContainer.elevator.moveTo(targetHeight);
        RobotContainer.elevatorArm.set(targetHeight.getArmAngle());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.elevator.moveTo(targetHeight);
        RobotContainer.elevatorArm.set(targetHeight.getArmAngle());

        if (Math.abs(RobotContainer.elevator.getCurrentVelocity()) >= 0.03 || RobotContainer.elevator.atGoal()) {
            lastMoveTime = Timer.getFPGATimestamp();
        } else {
            // If the elevator is not moving, check if it has been stationary for too long
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastMoveTime > 0.1) { // elevator stalled
                // Reset the elevator and arm to their initial positions
                RobotContainer.elevator.set(initialElevatorHeight);
                RobotContainer.elevatorArm.set(initialArmAngle);
                cancel();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.elevator.atGoal() && RobotContainer.elevatorArm.atGoal();
    }
}
