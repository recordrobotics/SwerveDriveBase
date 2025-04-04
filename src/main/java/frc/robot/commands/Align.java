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

  public static Command create(
      double tolerance, double rotTol, boolean forceUseTranslation, double maxDistance) {
    return create(tolerance, rotTol, forceUseTranslation, maxDistance, false, false);
  }

  public static Command create(
      double tolerance,
      double rotTol,
      boolean forceUseTranslation,
      double maxDistance,
      boolean useFirstStage) {
    return create(tolerance, rotTol, forceUseTranslation, maxDistance, useFirstStage, false);
  }

  public static Command create(
      double tolerance,
      double rotTol,
      boolean forceUseTranslation,
      double maxDistance,
      boolean useFirstStage,
      boolean useNear) {
    return new DeferredCommand(
        () -> {
          RobotAlignPose alignPose =
              RobotAlignPose.closestTo(
                  RobotContainer.poseTracker.getEstimatedPosition(), maxDistance);
          if (alignPose == null) return Commands.none();

          if (useFirstStage) {
            return new AlignToPose(
                alignPose.getFirstStagePose(),
                tolerance,
                rotTol,
                forceUseTranslation || alignPose.useTranslation());
          } else if (useNear) {
            return new AlignToPose(
                alignPose.getNearPose(),
                tolerance,
                rotTol,
                forceUseTranslation || alignPose.useTranslation());
          } else {
            return new AlignToPose(
                alignPose.getFarPose(),
                tolerance,
                rotTol,
                forceUseTranslation || alignPose.useTranslation());
          }
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createForReef(
      AutoScoreDirection direction, double tolerance, double rotTol) {
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

          return new AlignToPose(closest.getFarPose(), tolerance, rotTol, true);
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createForReefBackaway(
      AutoScoreDirection direction, double tolerance, double rotTol) {
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

          Pose2d backawayPose =
              closest.getFarPose().transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero));

          return new AlignToPose(backawayPose, tolerance, rotTol, true);
        },
        Set.of(RobotContainer.drivetrain));
  }
}
