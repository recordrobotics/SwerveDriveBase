package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.libraries.LimelightHelpers;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {

  private static final String name = "limelight";
  private static final double SCORE_DISTANCE = 3.0;
  private int numTags = 0;
  private double confidence = 0;
  private boolean hasVision = false;
  private boolean limelightConnected = false;
  private PoseEstimate currentEstimate = new PoseEstimate();
  private double currentConfidence = 9999999; // large number means less confident

  public Limelight() {
    LimelightHelpers.setPipelineIndex(name, 0);
  }

  @Override
  public void periodic() {
    confidence = 0;
    LimelightHelpers.SetRobotOrientation(
        name,
        RobotContainer.poseTracker.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    PoseEstimate measurement_m2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (measurement == null || measurement_m2 == null) {
      limelightConnected = false;
      return;
    } else {
      limelightConnected = true;
    }

    numTags = measurement.tagCount;

    if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
      if (measurement.avgTagDist
          < Units.feetToMeters(7)) { // 7 feet is where the MT1 (yellow) gets bad wiggles
        confidence = 0.65; // mt 1
      } else {
        confidence = 0.7; // mt 2
        measurement = measurement_m2;
      }
    }

    if (measurement
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
        > 2) {
      confidence = 0;
    }

    handleMeasurement(measurement, confidence);
    updateCrop();
  }

  private void updateCrop() {
    Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();
    double reefAngle =
        Math.atan2(
            pose.getTranslation().getY() - Constants.FieldPosition.ReefCenter.getPose().getY(),
            pose.getTranslation().getX() - Constants.FieldPosition.ReefCenter.getPose().getX());
    double processorAngle =
        Math.atan2(
            pose.getTranslation().getY() - Constants.FieldPosition.Processor.getPose().getY(),
            pose.getTranslation().getX() - Constants.FieldPosition.Processor.getPose().getX());

    Translation2d source12Pose = Constants.FieldConstants.SOURCE_12;
    Translation2d source13Pose = Constants.FieldConstants.SOURCE_13;
    Translation2d source1Pose = Constants.FieldConstants.SOURCE_1;
    Translation2d source2Pose = Constants.FieldConstants.SOURCE_2;

    double sourceAngle12 =
        Math.atan2(
            pose.getTranslation().getY() - source12Pose.getY(),
            pose.getTranslation().getX() - source12Pose.getX());
    double sourceAngle13 =
        Math.atan2(
            pose.getTranslation().getY() - source13Pose.getY(),
            pose.getTranslation().getX() - source13Pose.getX());

    double sourceAngle1 =
        Math.atan2(
            pose.getTranslation().getY() - source1Pose.getY(),
            pose.getTranslation().getX() - source1Pose.getX());

    double sourceAngle2 =
        Math.atan2(
            pose.getTranslation().getY() - source2Pose.getY(),
            pose.getTranslation().getX() - source2Pose.getX());

    double closestSourceDistance = pose.getTranslation().getDistance(source1Pose);
    double closestSourceAngle = sourceAngle1;
    Translation2d closestSourcePose = source1Pose;

    if (pose.getTranslation().getDistance(source2Pose) < closestSourceDistance) {
      closestSourceAngle = sourceAngle2;
      closestSourcePose = source2Pose;
      closestSourceDistance = pose.getTranslation().getDistance(source2Pose);
    }
    if (pose.getTranslation().getDistance(source12Pose) < closestSourceDistance) {
      closestSourceAngle = sourceAngle12;
      closestSourcePose = source12Pose;
      closestSourceDistance = pose.getTranslation().getDistance(source12Pose);
    }
    if (pose.getTranslation().getDistance(source13Pose) < closestSourceDistance) {
      closestSourceAngle = sourceAngle13;
      closestSourcePose = source13Pose;
      closestSourceDistance = pose.getTranslation().getDistance(source13Pose);
    }

    Logger.recordOutput("closestSource", closestSourcePose);
    Logger.recordOutput("sourceAngle", closestSourceAngle);

    if (pose.getTranslation().getDistance(Constants.FieldPosition.ReefCenter.getPose())
            < SCORE_DISTANCE
        && Math.abs(reefAngle - pose.getRotation().getRadians()) < Math.PI / 4) {
      setCrop(cropZones.REEF);
    } else if (pose.getTranslation().getDistance(Constants.FieldPosition.Processor.getPose())
            < SCORE_DISTANCE
        && Math.abs(processorAngle - pose.getRotation().getRadians()) < Math.PI / 4) {
      setCrop(cropZones.PROCESSOR);
    } else if (pose.getTranslation().getDistance(closestSourcePose) < SCORE_DISTANCE
        && MathUtil.angleModulus(closestSourceAngle - pose.getRotation().getRadians())
            < Math.PI / 4) {
      setCrop(cropZones.SOURCE);
    } else {
      setCrop(cropZones.Default);
    }
  }

  private void handleMeasurement(PoseEstimate estimate, double confidence) {
    if (confidence > 0) {
      hasVision = true;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentEstimate = estimate;
      currentConfidence = confidence;
    } else {
      hasVision = false;
      DashboardUI.Autonomous.setVisionPose(new Pose2d());
      currentConfidence = 9999999;
    }
  }

  private void setCrop(cropZones zone) {
    LimelightHelpers.setCropWindow(name, zone.x1, zone.x2, zone.y1, zone.y2);
    LimelightHelpers.SetFiducialDownscalingOverride(name, zone.scale);
  }

  private enum cropZones {
    // x1, x2, y1, y2, downscale factor
    REEF(-1, 1, -1, 0, 1),
    PROCESSOR(-1, 1, 0, 1, 1),
    SOURCE(-0.5, 0.5, -1, 0, 1),
    Default(-1, 1, -1, 1, 2);
    double x1, x2, y1, y2;
    float scale;

    cropZones(double x1, double x2, double y1, double y2, float scale) {
      this.x1 = x1;
      this.x2 = x2;
      this.y1 = y1;
      this.y2 = y2;
      this.scale = scale;
    }
  }

  public PoseEstimate getPoseEstimate() {
    return currentEstimate;
  }

  public double getConfidence() {
    return currentConfidence;
  }

  /** frees up all hardware allocations */
  public void close() {}

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setTagNum(() -> numTags);
    DashboardUI.Overview.setConfidence(() -> confidence);
    DashboardUI.Overview.setHasVision(() -> hasVision);
    DashboardUI.Overview.setLimelightConnected(() -> limelightConnected);
  }
}
