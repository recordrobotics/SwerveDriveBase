package frc.robot.utils.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera implements IVisionCamera {

    private int numTags = 0;
    private double confidence = 0;
    private boolean hasVision = false;
    private boolean connected = false;

    private double currentConfidence = 9999999; // large number means less confident
    private VisionCameraEstimate currentEstimate = new VisionCameraEstimate();
    private VisionCameraEstimate unsafeEstimate = new VisionCameraEstimate();

    private String name;

    private final double CONFIDENCE_CLOSE;
    private final double CONFIDENCE_FAR;
    private final double CLOSE_MAX_DIST;

    private final double MAX_POSE_ERROR;

    private CameraType type;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimatorClose;
    private final PhotonPoseEstimator photonEstimatorFar;

    private static final RawVisionFiducial[] ALL_SIM_TAGS = Constants.Game.APRILTAG_LAYOUT.getTags().stream()
            .map(v -> new RawVisionFiducial(v.ID, 0.1, 6, 7, 0))
            .toArray(RawVisionFiducial[]::new);

    public boolean isConnected() {
        return connected;
    }

    public String getName() {
        return name;
    }

    public CameraType getCameraType() {
        return type;
    }

    public boolean hasVision() {
        return hasVision;
    }

    public int getNumTags() {
        return numTags;
    }

    public VisionCameraEstimate getCurrentEstimate() {
        return currentEstimate;
    }

    public VisionCameraEstimate getUnsafeEstimate() {
        return unsafeEstimate;
    }

    public double getConfidence() {
        return currentConfidence;
    }

    public double getUnprocessedConfidence() {
        return confidence;
    }

    public PhotonVisionCamera(String name, CameraType type, Transform3d robotToCamera, double stdMultiplier) {
        this.name = name;
        this.type = type;
        this.CONFIDENCE_CLOSE = 0.65 * stdMultiplier;
        this.CONFIDENCE_FAR = 0.7 * stdMultiplier;
        this.CLOSE_MAX_DIST = Units.feetToMeters(7);
        this.MAX_POSE_ERROR = 10; // 10 meters

        camera = new PhotonCamera(name);

        photonEstimatorClose = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonEstimatorClose.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorFar = new PhotonPoseEstimator(
                Constants.Game.APRILTAG_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
        photonEstimatorFar.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL
                && Constants.RobotState.VISION_SIMULATION_MODE
                        == Constants.RobotState.VisionSimulationMode.PHOTON_SIM) {
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(
                    type.getDetectorWidth(), type.getDetectorHeight(), Rotation2d.fromDegrees(type.fov));
            cameraProp.setCalibError(type.pxError, type.pxErrorStdDev);
            cameraProp.setFPS(type.fps);
            cameraProp.setAvgLatencyMs(type.latencyMs);
            cameraProp.setLatencyStdDevMs(type.latencyStdDevMs);

            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableDrawWireframe(true);
            RobotContainer.visionSim.addCamera(cameraSim, robotToCamera);
        }
    }

    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    public void updateEstimation(boolean trust, boolean ignore) {
        confidence = 0;

        Optional<VisionCameraEstimate> measurement_close_opt;
        Optional<VisionCameraEstimate> measurement_far_opt;

        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL
                || Constants.RobotState.VISION_SIMULATION_MODE
                        == Constants.RobotState.VisionSimulationMode.PHOTON_SIM) {
            photonEstimatorClose.addHeadingData(
                    Timer.getTimestamp(),
                    new Rotation3d(
                            0,
                            0,
                            RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getRotation()
                                    .getRadians()));
            photonEstimatorFar.addHeadingData(
                    Timer.getTimestamp(),
                    new Rotation3d(
                            0,
                            0,
                            RobotContainer.poseSensorFusion
                                    .getEstimatedPosition()
                                    .getRotation()
                                    .getRadians()));

            measurement_close_opt = getEstimatedGlobalPose(photonEstimatorClose, false);
            measurement_far_opt = getEstimatedGlobalPose(photonEstimatorFar, true);
        } else {
            Pose2d maplePose = RobotContainer.model.getRobot();
            if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.MAPLE_NOISE) {
                maplePose = SimpleMath.poseNoise(maplePose, 0.001, 0.001);
            }

            measurement_close_opt = Optional.of(new VisionCameraEstimate(
                    maplePose, Timer.getTimestamp(), 0.01, ALL_SIM_TAGS.length, 6, 0.1, ALL_SIM_TAGS, false));
            measurement_far_opt = Optional.of(new VisionCameraEstimate(
                    maplePose, Timer.getTimestamp(), 0.01, ALL_SIM_TAGS.length, 6, 0.1, ALL_SIM_TAGS, true));
        }

        if (!camera.isConnected()) {
            connected = false;
            return;
        } else {
            connected = true;
        }

        if (measurement_close_opt.isEmpty() && measurement_far_opt.isEmpty()) {
            hasVision = false;
            currentConfidence = 9999999;
            confidence = 0;
            return;
        }

        VisionCameraEstimate measurement_close =
                measurement_close_opt.isEmpty() ? measurement_far_opt.get() : measurement_close_opt.get();
        VisionCameraEstimate measurement_far =
                measurement_far_opt.isEmpty() ? measurement_close_opt.get() : measurement_far_opt.get();

        numTags = measurement_close.tagCount;

        if (!DashboardUI.Autonomous.getForceMT1()) {
            if (measurement_close.tagCount > 0 && SimpleMath.isPoseInField(measurement_close.pose)) {
                if (measurement_close.avgTagDist < CLOSE_MAX_DIST) {
                    confidence = CONFIDENCE_CLOSE;
                } else {
                    confidence = CONFIDENCE_FAR;
                    measurement_close = measurement_far;
                }
            }
        } else { // reseting robot pose before match
            confidence = CONFIDENCE_CLOSE;
        }

        unsafeEstimate = measurement_close;

        if (measurement_close
                        .pose
                        .getTranslation()
                        .getDistance(RobotContainer.poseSensorFusion
                                .getEstimatedPosition()
                                .getTranslation())
                > MAX_POSE_ERROR) {
            confidence = 0;
        }

        if (confidence > 0) {
            hasVision = true;
            currentEstimate = measurement_close;
            currentConfidence = confidence;
            RobotContainer.poseSensorFusion.addVisionMeasurement(
                    currentEstimate.pose,
                    currentEstimate.timestampSeconds,
                    VecBuilder.fill(
                            currentConfidence,
                            currentConfidence,
                            trust
                                    ? Constants.PhotonVision.ROT_STD_DEV_WHEN_TRUSTING
                                    : 9999999) // some influence of limelight pose rotation
                    );
        } else {
            hasVision = false;
            currentConfidence = 9999999;
        }
    }

    public void logValues(String id) {
        String prefix = "PhotonCamera/" + id + "/";
        Logger.recordOutput(prefix + "Pose", unsafeEstimate.pose);
        Logger.recordOutput(prefix + "NumTags", numTags);
        Logger.recordOutput(prefix + "Confidence", currentConfidence);
        Logger.recordOutput(prefix + "HasVision", hasVision);
        Logger.recordOutput(prefix + "Connected", connected);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<VisionCameraEstimate> getEstimatedGlobalPose(PhotonPoseEstimator estimator, boolean isConstrained) {
        Optional<VisionCameraEstimate> visionEst = Optional.empty();
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> est_opt = estimator.update(change);

            if (est_opt.isPresent()) {
                EstimatedRobotPose est = est_opt.get();
                int numTags = est.targetsUsed.size();

                double avgTagDist = 0;
                double avgTagArea = 0;

                List<RawVisionFiducial> rawFiducials = new ArrayList<>(numTags);
                for (PhotonTrackedTarget target : est.targetsUsed) {
                    double dist =
                            target.getBestCameraToTarget().getTranslation().getNorm();
                    double distRobot = target.getBestCameraToTarget()
                            .plus(estimator.getRobotToCameraTransform())
                            .getTranslation()
                            .getNorm();
                    avgTagDist += dist;
                    avgTagArea += target.getArea();

                    RawVisionFiducial rawFiducial = new RawVisionFiducial(
                            target.fiducialId, target.getArea(), dist, distRobot, target.getPoseAmbiguity());
                    rawFiducials.add(rawFiducial);
                }
                avgTagDist /= numTags;
                avgTagArea /= numTags;

                VisionCameraEstimate estimation = new VisionCameraEstimate(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        type.latencyMs,
                        numTags,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials.toArray(new RawVisionFiducial[0]),
                        isConstrained);

                visionEst = Optional.of(estimation);
            } else {
                visionEst = Optional.empty();
            }
        }
        return visionEst;
    }

    public void close() {
        // camera.close();
    }

    public void kill() {}
}
