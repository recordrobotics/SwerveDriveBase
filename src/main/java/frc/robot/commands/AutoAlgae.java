package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;

public class AutoAlgae extends SequentialCommandGroup {

    private boolean alignTimeout = false;
    private static boolean cancelCommand = false;
    private static boolean isRunning = false;

    public static void performCancel() {
        cancelCommand = true;
    }

    public static boolean isRunning() {
        return isRunning;
    }

    public static void stopRunning() {
        isRunning = false;
    }

    public static Command algaeGrabCommand(ElevatorHeight targetHeight) {
        return new ElevatorMove(targetHeight)
                .andThen(new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                                RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_GROUND);
                            else RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_REEF);
                        },
                        RobotContainer.elevatorHead));
    }

    public AutoAlgae(AlgaePosition reefPole) {
        addCommands(
                new InstantCommand(() -> {
                    alignTimeout = false;
                    cancelCommand = false;
                    isRunning = true;
                }),
                GameAlign.makeAlignWithCommand(
                                (usePath, useAlign) -> AlgaeAlign.alignTarget(
                                                reefPole, usePath, useAlign, false, 2.0, 1.0)
                                        .asProxy(),
                                () -> {
                                    Pose2d pose = reefPole.getPose();

                                    double dist = RobotContainer.poseSensorFusion
                                            .getEstimatedPosition()
                                            .getTranslation()
                                            .getDistance(pose.getTranslation());

                                    return (dist < Constants.Align.CLEARANCE_MAX
                                                    && dist > Constants.Align.CLEARANCE_MIN)
                                            || alignTimeout;
                                },
                                () -> algaeGrabCommand(reefPole.getLevel().getHeight())
                                        .asProxy(),
                                () -> {
                                    Pose2d pose = reefPole.getPose();

                                    double dist = RobotContainer.poseSensorFusion
                                            .getEstimatedPosition()
                                            .getTranslation()
                                            .getDistance(pose.getTranslation());

                                    return dist > Constants.Align.CLEARANCE_MAX;
                                })
                        .onlyWhile(() -> RobotState.isAutonomous() || !cancelCommand),
                new WaitUntilCommand(() -> {
                    Pose2d pose = reefPole.getPose();

                    double dist = RobotContainer.poseSensorFusion
                            .getEstimatedPosition()
                            .getTranslation()
                            .getDistance(pose.getTranslation());

                    return cancelCommand || (dist > 1.0);
                }),
                new ElevatorMoveThenAlgaeGrabEnd(reefPole.getLevel().getHeight(), true).asProxy());
    }
}
