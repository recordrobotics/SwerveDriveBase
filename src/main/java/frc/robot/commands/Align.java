package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.control.AbstractControl.AutoScoreDirection;
import java.util.Set;

public class Align {
  public static Command create(double maxDistance, boolean useFirstStage, boolean useNear) {
    return new DeferredCommand(
        () -> {
          RobotAlignPose alignPose =
              RobotAlignPose.closestTo(
                  RobotContainer.poseTracker.getEstimatedPosition(), maxDistance);
          if (alignPose == null) return Commands.none();

          if (useFirstStage) {
            return new AlignToPose(alignPose.getFirstStagePose(), alignPose.useTranslation());
          } else if (useNear) {
            return new AlignToPose(alignPose.getNearPose(), alignPose.useTranslation());
          } else {
            return new AlignToPose(alignPose.getFarPose(), alignPose.useTranslation());
          }
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createForReef(AutoScoreDirection direction) {
    return createForReef(direction, false);
  }

  public static Command createForReefBackaway(AutoScoreDirection direction) {
    return createForReef(direction, true);
  }

  public static Command createForReef(AutoScoreDirection direction, boolean isBackaway) {
    return new DeferredCommand(
        () -> {
          Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();

          double maxDistance = 2.5;
          RobotAlignPose closest = null;
          double closestDistance = Double.MAX_VALUE;
          for (RobotAlignPose align :
              direction == AutoScoreDirection.Left
                  ? RobotAlignPose.leftReefPoses
                  : RobotAlignPose.rightReefPoses) {
            double distance =
                align.getFarPose().getTranslation().getDistance(pose.getTranslation());
            if (distance <= maxDistance && distance < closestDistance) {
              closest = align;
              closestDistance = distance;
            }
          }

          if (closest == null) return Commands.none();

          Pose2d targetPose = closest.getFarPose();
          if (isBackaway) {
            targetPose = targetPose.transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero));
          }

          return new AlignToPose(targetPose, true);
        },
        Set.of(RobotContainer.drivetrain));
  }
}
