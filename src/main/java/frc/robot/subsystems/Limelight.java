package frc.robot.subsystems;

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
  public static class LimelightData {
    public int numTags = 0;
    private double confidence = 0;
    public boolean hasVision = false;
    public boolean limelightConnected = false;

    public double currentConfidence = 9999999; // large number means less confident
    public PoseEstimate currentEstimate = new PoseEstimate();
    public PoseEstimate unsafeEstimate = new PoseEstimate();

    public String name;

    private final double MT1_CONFIDENCE;
    private final double MT2_CONFIDENCE;

    private final double MT1_MAX_DIST;

    private final double MAX_POSE_ERROR;

    public LimelightData(String name) {
      this.name = name;
      this.MT1_CONFIDENCE = 0.65;
      this.MT2_CONFIDENCE = 0.7;
      this.MT1_MAX_DIST =
          Units.feetToMeters(7); // 7 feet is where the MT1 (yellow) gets bad wiggles
      this.MAX_POSE_ERROR = 5; // 5 meters
    }

    public LimelightData(
        String name,
        double mt1_confidence,
        double mt2_confidence,
        double mt1_max_dist,
        double max_pose_error) {
      this.name = name;
      this.MT1_CONFIDENCE = mt1_confidence;
      this.MT2_CONFIDENCE = mt2_confidence;
      this.MT1_MAX_DIST = mt1_max_dist;
      this.MAX_POSE_ERROR = max_pose_error;
    }

    public void setPipeline(int pipeline) {
      LimelightHelpers.setPipelineIndex(name, pipeline);
    }

    public void updateEstimation() {
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

      if (!DashboardUI.Autonomous.getForceMT1()) {
        if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
          if (measurement.avgTagDist < MT1_MAX_DIST) {
            confidence = MT1_CONFIDENCE;
          } else {
            confidence = MT2_CONFIDENCE;
            measurement = measurement_m2;
          }
        }
      } else {
        confidence = MT1_CONFIDENCE;
      }

      unsafeEstimate = measurement;

      if (measurement
              .pose
              .getTranslation()
              .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
          > MAX_POSE_ERROR) {
        confidence = 0;
      }

      if (confidence > 0) {
        hasVision = true;
        currentEstimate = measurement;
        currentConfidence = confidence;
      } else {
        hasVision = false;
        currentConfidence = 9999999;
      }
    }

    public void logValues(String id) {
      String prefix = "Limelight/" + id + "/";
      Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose);
      Logger.recordOutput(prefix + "NumTags", numTags);
      Logger.recordOutput(prefix + "Confidence", currentConfidence);
      Logger.recordOutput(prefix + "HasVision", hasVision);
      Logger.recordOutput(prefix + "LimelightConnected", limelightConnected);
    }
  }

  private LimelightData left = new LimelightData(Constants.Limelight.LIMELIGHT_LEFT_NAME);
  private LimelightData center = new LimelightData(Constants.Limelight.LIMELIGHT_CENTER_NAME);

  public Limelight() {
    left.setPipeline(0);
    center.setPipeline(0);
  }

  @Override
  public void periodic() {
    left.updateEstimation();
    center.updateEstimation();

    DashboardUI.Autonomous.setVisionPoseLeft(left.unsafeEstimate.pose);

    left.logValues("Left");
    center.logValues("Center");
  }

  public LimelightData getLeft() {
    return left;
  }

  public LimelightData getCenter() {
    return center;
  }

  /** frees up all hardware allocations */
  public void close() {}

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setTagNumLeft(() -> left.numTags);
    DashboardUI.Overview.setConfidenceLeft(() -> left.confidence);
    DashboardUI.Overview.setHasVisionLeft(() -> left.hasVision);
    DashboardUI.Overview.setLimelightConnectedLeft(() -> left.limelightConnected);

    DashboardUI.Overview.setTagNumCenter(() -> center.numTags);
    DashboardUI.Overview.setConfidenceCenter(() -> center.confidence);
    DashboardUI.Overview.setHasVisionCenter(() -> center.hasVision);
    DashboardUI.Overview.setLimelightConnectedCenter(() -> center.limelightConnected);
  }
}
