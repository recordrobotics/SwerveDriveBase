package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.real.NavSensorReal;
import frc.robot.subsystems.io.sim.NavSensorSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.IndependentSwervePoseEstimator;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.camera.IVisionCamera;
import frc.robot.utils.camera.VisionCameraEstimate.RawVisionFiducial;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

public class PoseSensorFusion extends ManagedSubsystemBase {

    public static final double MAX_MEASUREMENT_STD_DEVS = 9_999_999;

    private static final double ISPE_STD_DEV = 0.7;

    private static final double DEFAULT_DEBOUNCE_TIME = 0.5;

    public final NavSensor nav;

    private SwerveDrivePoseEstimator poseFilter;
    private IndependentSwervePoseEstimator independentPoseEstimator;

    private final Set<IVisionCamera> cameras = Set.of();

    private final HashSet<VisionDebouncer> visionDebouncers = new HashSet<>();

    private ConcurrentSkipListSet<DeferredPoseEstimation> deferredPoseEstimations =
            new ConcurrentSkipListSet<>((a, b) -> Double.compare(a.timestampSeconds, b.timestampSeconds));

    private boolean useISPE;

    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private Future<?> calculationFuture = null;

    private double updateTimestamp;
    private Rotation2d updateNav;
    private SwerveModulePosition[] updatePositions;

    private int nextVisionId = 1;

    public PoseSensorFusion() {
        nav = new NavSensor(
                Constants.RobotState.getMode() == Mode.REAL
                        ? new NavSensorReal()
                        : new NavSensorSim(RobotContainer.drivetrain
                                .getSwerveDriveSimulation()
                                .getGyroSimulation()));
        nav.resetAngleAdjustment();

        poseFilter = new SwerveDrivePoseEstimator(
                RobotContainer.drivetrain.getKinematics(),
                nav.getAdjustedAngle(),
                getModulePositions(),
                DashboardUI.Overview.getStartingLocation().getPose());

        independentPoseEstimator = new IndependentSwervePoseEstimator(
                getEstimatedPosition(),
                new SwerveModule[] {
                    RobotContainer.drivetrain.getFrontLeftModule(),
                    RobotContainer.drivetrain.getFrontRightModule(),
                    RobotContainer.drivetrain.getBackLeftModule(),
                    RobotContainer.drivetrain.getBackRightModule()
                },
                new Translation2d[] {
                    Constants.Swerve.FRONT_LEFT_WHEEL_LOCATION,
                    Constants.Swerve.FRONT_RIGHT_WHEEL_LOCATION,
                    Constants.Swerve.BACK_LEFT_WHEEL_LOCATION,
                    Constants.Swerve.BACK_RIGHT_WHEEL_LOCATION
                });

        SmartDashboard.putBoolean("Overview/UseISPE", true);
    }

    public record DeferredPoseEstimation(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        deferredPoseEstimations.add(
                new DeferredPoseEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    }

    public void startCalculation() {
        deferredPoseEstimations.clear();

        useISPE = SmartDashboard.getBoolean("Overview/UseISPE", true);

        calculationFuture = executor.submit(this::calculationLoop);
    }

    public void endCalculation() {
        if (updatePositions != null) {
            poseFilter.updateWithTime(updateTimestamp, updateNav, updatePositions);
        }

        if (calculationFuture != null) {
            try {
                calculationFuture.get();
            } catch (ExecutionException e) {
                e.printStackTrace();
            } catch (InterruptedException e) {
                e.printStackTrace();
                Thread.currentThread().interrupt();
            }
        }

        while (!deferredPoseEstimations.isEmpty()) {
            DeferredPoseEstimation estimation = deferredPoseEstimations.pollFirst();
            poseFilter.addVisionMeasurement(
                    estimation.visionRobotPoseMeters, estimation.timestampSeconds, estimation.visionMeasurementStdDevs);
        }

        updateDashboard();

        for (VisionDebouncer debouncer : visionDebouncers) {
            debouncer.update();
        }

        Logger.recordOutput("SwerveEstimations", independentPoseEstimator.getEstimatedModulePositions());
        Logger.recordOutput("RobotEstimations", independentPoseEstimator.getEstimatedRobotPoses());
        Logger.recordOutput("RobotEstimation", independentPoseEstimator.getEstimatedRobotPose());
    }

    @Override
    public void periodicManaged() {
        /* logic is asynchronous */
    }

    public void calculationLoop() {
        updateTimestamp = Timer.getTimestamp();
        updateNav = nav.getAdjustedAngle();
        updatePositions = getModulePositions();

        if (cameras.stream().anyMatch(v -> v.hasVision())) {
            // when vision is correcting the pose, have that override the independent pose estimator
            independentPoseEstimator.reset(getEstimatedPosition());
            independentPoseEstimator.update(getEstimatedPosition().getRotation());
        } else {
            independentPoseEstimator.update(getEstimatedPosition().getRotation());

            // when no vision use independent pose estimator to correct pose
            if (useISPE) {
                addVisionMeasurement(
                        independentPoseEstimator.getEstimatedRobotPose(),
                        Timer.getTimestamp(),
                        VecBuilder.fill(ISPE_STD_DEV, ISPE_STD_DEV, MAX_MEASUREMENT_STD_DEVS));
            }
        }
    }

    private void updateDashboard() {
        for (IVisionCamera camera : cameras) {
            DashboardUI.Overview.setVisionPose(
                    camera.getName(), camera.getUnsafeEstimate().pose());
        }

        SmartDashboard.putNumber(
                "pose", poseFilter.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("gyro", nav.getAdjustedAngle().getDegrees());
        DashboardUI.Overview.setRobotPose(poseFilter.getEstimatedPosition());
    }

    private static SwerveModulePosition[] getModulePositions() {
        return RobotContainer.drivetrain.getModulePositions();
    }

    @AutoLogLevel(key = "Odometry/Robot", level = Level.REAL)
    public Pose2d getEstimatedPosition() {
        return poseFilter.getEstimatedPosition();
    }

    public Pose2d getEstimatedPositionAt(double timestamp) {
        Optional<Pose2d> sample = poseFilter.sampleAt(timestamp);
        if (sample.isEmpty()) return getEstimatedPosition();
        else return sample.get();
    }

    /** Similar to resetPose but adds an argument for the initial pose */
    public void setToPose(Pose2d pose) {
        poseFilter.resetPosition(nav.getAdjustedAngle(), getModulePositions(), pose);
        independentPoseEstimator.reset(pose);
    }

    /** Resets the field relative position of the robot (mostly for testing). */
    public void resetStartingPose() {
        setToPose(DashboardUI.Overview.getStartingLocation().getPose());
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            RobotContainer.drivetrain
                    .getSwerveDriveSimulation()
                    .setSimulationWorldPose(
                            DashboardUI.Overview.getStartingLocation().getPose());
        }
    }

    public void resetToVision() {
        cameras.stream()
                .filter(IVisionCamera::hasVision)
                .sorted((a, b) -> Double.compare(
                        a.getCurrentEstimate().avgTagDist(),
                        b.getCurrentEstimate().avgTagDist()))
                .findFirst()
                .ifPresentOrElse(
                        camera -> setToPose(camera.getCurrentEstimate().pose()),
                        () -> DriverStation.reportWarning("No camera has vision!", false));
    }

    /**
     * Resets the pose to face elevator away from driverstation, while keeping translation the same
     */
    public void resetDriverPose() {
        Pose2d current = getEstimatedPosition();
        setToPose(new Pose2d(
                current.getTranslation(),
                DriverStationUtils.getCurrentAlliance() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d(0)));
    }

    public enum CameraTarget {
        @SuppressWarnings("java:S3878")
        NONE(new IVisionCamera[0]);

        private Set<IVisionCamera> cameras;

        CameraTarget(IVisionCamera... cameras) {
            this.cameras = Set.of(cameras);
        }

        @SuppressWarnings("java:S1144")
        CameraTarget(CameraTarget... targets) {
            List<IVisionCamera> cameraList = new ArrayList<>();
            for (CameraTarget target : targets) {
                for (IVisionCamera camera : target.cameras) {
                    cameraList.add(camera);
                }
            }
            this.cameras = Set.of(cameraList.toArray(new IVisionCamera[0]));
        }

        boolean contains(IVisionCamera camera) {
            return cameras.contains(camera);
        }

        boolean contains(IVisionCamera... cameras) {
            return this.cameras.containsAll(Set.of(cameras));
        }
    }

    public VisionDebouncer registerVisionCheck(CameraTarget camera) {
        return registerVisionCheck(camera, DEFAULT_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    }

    public VisionDebouncer registerVisionCheck(
            CameraTarget camera, double debounceTime, Debouncer.DebounceType debounceType) {
        return registerVisionCheck(camera, null, debounceTime, debounceType);
    }

    public VisionDebouncer registerVisionCheck(CameraTarget camera, int... tagIds) {
        return registerVisionCheck(camera, tagIds, DEFAULT_DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);
    }

    public VisionDebouncer registerVisionCheck(
            CameraTarget camera, int[] tagIds, double debounceTime, Debouncer.DebounceType debounceType) {
        VisionDebouncer vision = new VisionDebouncer(nextVisionId, debounceTime, debounceType, camera, tagIds);
        visionDebouncers.add(vision);
        return vision;
    }

    public void releaseVisionCheck(VisionDebouncer visionDebouncer) {
        visionDebouncers.remove(visionDebouncer);
    }

    @Override
    public void close() throws Exception {
        nav.close();
    }

    public Set<IVisionCamera> getCameras() {
        return cameras;
    }

    public static final class VisionDebouncer {
        private final Debouncer debouncer;
        private CameraTarget camera;
        private int[] tagIds;

        private final int id;

        private boolean result = false;
        private double lastAccessTime;

        private boolean persistent = false;

        private VisionDebouncer(
                int id, double debounceTime, Debouncer.DebounceType debounceType, CameraTarget camera, int[] tagIds) {
            this.id = id;
            this.camera = camera;
            this.tagIds = tagIds;

            debouncer = new Debouncer(0, debounceType);
            debouncer.calculate(hasVisionRaw());
            debouncer.setDebounceTime(debounceTime);

            lastAccessTime = Timer.getTimestamp();
        }

        private boolean hasVisionRaw() {
            boolean rawInput = hasAnyVision();
            if (rawInput && tagIds != null) {
                rawInput = hasRequiredTags();
            }
            return rawInput;
        }

        private boolean hasAnyVision() {
            for (IVisionCamera vis : RobotContainer.poseSensorFusion.getCameras()) {
                if (camera.contains(vis) && vis.hasVision()) {
                    return true;
                }
            }
            return false;
        }

        private boolean hasRequiredTags() {
            ArrayList<Integer> visionTags = collectVisionTags();
            for (int tagId : tagIds) {
                if (!visionTags.contains(tagId)) {
                    return false;
                }
            }
            return true;
        }

        private ArrayList<Integer> collectVisionTags() {
            ArrayList<Integer> visionTags = new ArrayList<>();
            for (IVisionCamera vis : RobotContainer.poseSensorFusion.getCameras()) {
                if (camera.contains(vis)) {
                    for (RawVisionFiducial tag : vis.getCurrentEstimate().rawFiducials()) {
                        visionTags.add(tag.id());
                    }
                }
            }
            return visionTags;
        }

        private void update() {
            boolean rawInput = hasVisionRaw();

            result = debouncer.calculate(rawInput);

            double timeSinceLastAccessed = Timer.getTimestamp() - lastAccessTime;
            if (!persistent && timeSinceLastAccessed > 1.0) {
                DriverStation.reportWarning(
                        "Detected abandoned "
                                + toString()
                                + " last accessed "
                                + (int) Math.floor(timeSinceLastAccessed)
                                + " seconds ago.",
                        false);
            }
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("VisionDebouncer{");
            sb.append("id=").append(id);
            sb.append(", camera=").append(camera);
            sb.append(", tagIds=");
            if (tagIds != null) {
                for (int tagId : tagIds) {
                    sb.append(tagId).append(" ");
                }
            }
            sb.append(", value=").append(result);
            sb.append('}');
            return sb.toString();
        }

        public boolean hasVision() {
            lastAccessTime = Timer.getTimestamp();
            return result;
        }

        public void setCamera(CameraTarget camera) {
            this.camera = camera;
        }

        public VisionDebouncer withCamera(CameraTarget camera) {
            setCamera(camera);
            return this;
        }

        public void setTagIds(int[] tagIds) {
            this.tagIds = tagIds;
        }

        public VisionDebouncer withTagIds(int[] tagIds) {
            setTagIds(tagIds);
            return this;
        }

        public CameraTarget getCamera() {
            return camera;
        }

        public int[] getTagIds() {
            return tagIds;
        }

        public void setDebounceTime(double time) {
            debouncer.setDebounceTime(time);
        }

        public void setDebounceType(Debouncer.DebounceType debounceType) {
            debouncer.setDebounceType(debounceType);
        }

        public VisionDebouncer withDebounceTime(double time) {
            setDebounceTime(time);
            return this;
        }

        public VisionDebouncer withDebounceType(Debouncer.DebounceType debounceType) {
            setDebounceType(debounceType);
            return this;
        }

        public double getDebounceTime() {
            return debouncer.getDebounceTime();
        }

        public Debouncer.DebounceType getDebounceType() {
            return debouncer.getDebounceType();
        }

        public int getId() {
            return id;
        }

        public void release() {
            RobotContainer.poseSensorFusion.releaseVisionCheck(this);
        }

        public boolean isPersistent() {
            return persistent;
        }

        public void setPersistent(boolean persistent) {
            this.persistent = persistent;
        }

        public void setPersistent() {
            setPersistent(true);
        }

        public VisionDebouncer withPersistent() {
            setPersistent();
            return this;
        }

        public VisionDebouncer withPersistent(boolean persistent) {
            setPersistent(persistent);
            return this;
        }
    }
}
