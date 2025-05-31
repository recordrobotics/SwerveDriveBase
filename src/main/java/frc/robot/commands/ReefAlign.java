package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import java.util.Set;
import java.util.function.Supplier;

public class ReefAlign {

  public static Command alignClosest(
      boolean usePath, boolean useAlign, boolean repeatedly, boolean useAdditionalOffset) {
    return alignClosest(
        () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(),
        usePath,
        useAlign,
        repeatedly,
        useAdditionalOffset);
  }

  public static Command alignClosest(
      Supplier<CoralLevel> level,
      boolean usePath,
      boolean useAlign,
      boolean repeatedly,
      boolean useAdditionalOffset) {
    return new DeferredCommand(
        () -> {
          CoralPosition alignPose =
              IGamePosition.closestTo(
                  RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

          if (alignPose
                  .getPose(level.get())
                  .getTranslation()
                  .getDistance(
                      RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation())
              > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return Commands.none();

          return alignTarget(alignPose, level, usePath, useAlign, repeatedly, useAdditionalOffset);
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command alignTarget(
      CoralPosition pole,
      Supplier<CoralLevel> level,
      boolean usePath,
      boolean useAlign,
      boolean repeatedly,
      boolean useAdditionalOffset) {
    return GameAlign.alignTarget(
        () ->
            useAdditionalOffset
                ? pole.getPose(level.get())
                    .transformBy(
                        new Transform2d(-Constants.Align.ADDITIONAL_OFFSET, 0, Rotation2d.kZero))
                : pole.getPose(level.get()),
        level.get() == CoralLevel.L1
            ? new Transform2d(0, -Constants.Align.L1_CLEARANCE_MIN, Rotation2d.kZero)
            : new Transform2d(-Constants.Align.CLEARANCE_MIN, 0, Rotation2d.kZero),
        usePath,
        useAlign,
        repeatedly);
  }
}
