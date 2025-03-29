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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase implements ShuffleboardPublisher {
  @AutoLogOutput private int numTags = 0;

  private double confidence = 0;

  @AutoLogOutput private boolean hasVision = false;
  private String name = Constants.Limelight.LIMELIGHT_NAME;

  @AutoLogOutput private boolean limelightConnected = false;

  private double currentConfidence = 9999999; // large number means less confident
  private PoseEstimate currentEstimate = new PoseEstimate();

  private PoseEstimate unsafeEstimate = new PoseEstimate();

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

    if (!DashboardUI.Autonomous.getForceMT1()) {
      if (measurement.tagCount > 0 && SimpleMath.isPoseInField(measurement.pose)) {
        if (measurement.avgTagDist
            < Units.feetToMeters(7)) { // 7 feet is where the MT1 (yellow) gets bad wiggles
          confidence = 0.65; // mt 1
        } else {
          confidence = 0.7; // mt 2
          measurement = measurement_m2;
        }
      }
    } else {
      confidence = 0.65; // mt 1
    }

    unsafeEstimate = measurement;

    if (measurement
            .pose
            .getTranslation()
            .getDistance(RobotContainer.poseTracker.getEstimatedPosition().getTranslation())
        > 5) {
      confidence = 0;
    }

    handleMeasurement(measurement, confidence);
  }

  private void handleMeasurement(PoseEstimate estimate, double confidence) {
    if (confidence > 0) {
      hasVision = true;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentEstimate = estimate;
      currentConfidence = confidence;
      Logger.recordOutput("Limelight/Pose", estimate.pose);
    } else {
      hasVision = false;
      DashboardUI.Autonomous.setVisionPose(estimate.pose);
      currentConfidence = 9999999;
      Logger.recordOutput("Limelight/Pose", estimate.pose);
    }
  }

  public PoseEstimate getPoseEstimate() {
    return currentEstimate;
  }

  public PoseEstimate getUnsafePoseEstimate() {
    return unsafeEstimate;
  }

  @AutoLogOutput
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
