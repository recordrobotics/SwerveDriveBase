package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import java.util.Set;

public class AutoScore extends SequentialCommandGroup {

    private Pose2d backawayTargetPose = null;
    private boolean isL1 = false;

    private CoralLevel getLevel() {
        return isL1 && !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                ? CoralLevel.L1
                : DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel();
    }

    public AutoScore(CoralPosition reefPole) {
        addCommands(
                new InstantCommand(() -> {
                    isL1 = DashboardUI.Overview.getControl().getReefLevelSwitchValue() == ReefLevelSwitchValue.L1;
                }),
                GameAlign.makeAlignWithCommand(
                        (usePath, useAlign) -> ReefAlign.alignTarget(
                                        reefPole, this::getLevel, usePath, useAlign, true, 2.0, 1.0, false)
                                .asProxy(),
                        () -> {
                            CoralLevel level = getLevel();
                            Pose2d pose = reefPole.getPose(level);

                            double clearanceMin = (level == CoralLevel.L1
                                    ? Constants.Align.L1_CLEARANCE_MIN
                                    : level == CoralLevel.L4
                                            ? Constants.Align.L4_CLEARANCE_MIN
                                            : Constants.Align.CLEARANCE_MIN);
                            double clearanceMax = (level == CoralLevel.L1
                                    ? Constants.Align.L1_CLEARANCE_MAX
                                    : level == CoralLevel.L4
                                            ? Constants.Align.L4_CLEARANCE_MAX
                                            : Constants.Align.CLEARANCE_MAX);

                            double dist = RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(pose.getTranslation());

                            return (dist < clearanceMax && dist > clearanceMin);
                        },
                        () -> Commands.either(
                                new DeferredCommand(
                                        () -> new ElevatorMove(DashboardUI.Overview.getControl()
                                                        .getReefLevelSwitchValue()
                                                        .toCoralLevel()
                                                        .getHeight())
                                                .asProxy(),
                                        Set.of()),
                                new CoralIntakeMoveL1().asProxy(),
                                () -> getLevel() != CoralLevel.L1),
                        () -> {
                            CoralLevel level = getLevel();
                            Pose2d pose = reefPole.getPose(level);

                            double clearanceMax = (level == CoralLevel.L1
                                    ? Constants.Align.L1_CLEARANCE_MAX
                                    : level == CoralLevel.L4
                                            ? Constants.Align.L4_CLEARANCE_MAX
                                            : Constants.Align.CLEARANCE_MAX);

                            double dist = RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(pose.getTranslation());

                            return dist > clearanceMax;
                        }),
                Commands.either(
                        new WaitUntilCommand(() -> RobotState.isAutonomous()
                                        || !DashboardUI.Overview.getControl().getAutoScore())
                                .andThen(Commands.either(
                                        new CoralShoot()
                                                .andThen(() -> backawayTargetPose = RobotContainer.poseSensorFusion
                                                        .getEstimatedPosition()
                                                        .transformBy(new Transform2d(-0.6, 0, Rotation2d.kZero)))
                                                .andThen(GameAlign.alignTarget(
                                                                () -> backawayTargetPose,
                                                                Transform2d.kZero,
                                                                false,
                                                                true,
                                                                false,
                                                                2.0,
                                                                2.0) // back away
                                                        .asProxy()
                                                        .finallyDo(() -> RobotContainer.drivetrain.kill())
                                                        .alongWith(new DeferredCommand(
                                                                        () -> new WaitCommand(
                                                                                RobotContainer.elevator
                                                                                                        .getNearestHeight()
                                                                                                == ElevatorHeight.L4
                                                                                        ? 1.0
                                                                                        : 0),
                                                                        Set.of())
                                                                .andThen(
                                                                        new ElevatorMove(ElevatorHeight.BOTTOM)
                                                                                .asProxy())
                                                                .onlyIf(() -> !CoralShoot.failedToShoot))),
                                        new CoralIntakeShootL1().asProxy(),
                                        () -> getLevel() != CoralLevel.L1)),
                        Commands.either(
                                new ElevatorMove(ElevatorHeight.BOTTOM).asProxy(),
                                new InstantCommand(
                                                () -> {
                                                    RobotContainer.coralIntake.set(CoralIntakeState.UP);
                                                },
                                                RobotContainer.coralIntake)
                                        .asProxy(),
                                () -> getLevel() != CoralLevel.L1),
                        () -> RobotState.isAutonomous()
                                || DashboardUI.Overview.getControl().getAutoScore()
                                || RobotContainer.elevator.getNearestHeight()
                                        == DashboardUI.Overview.getControl()
                                                .getReefLevelSwitchValue()
                                                .toCoralLevel()
                                                .getHeight()));
    }
}
