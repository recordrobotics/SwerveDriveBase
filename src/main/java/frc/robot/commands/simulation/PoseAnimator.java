package frc.robot.commands.simulation;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PoseAnimator extends Command {

    /** The timer used for waiting. */
    protected Timer m_timer = new Timer();

    private final double m_duration;

    private final Pose3d m_startPose;
    private final Supplier<Pose3d> m_endPoseSupplier;

    private final Consumer<Pose3d> m_poseConsumer;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public PoseAnimator(
            Pose3d startPose, Supplier<Pose3d> endPoseSupplier, Consumer<Pose3d> poseConsumer, double seconds) {
        m_duration = seconds;
        m_startPose = startPose;
        m_endPoseSupplier = endPoseSupplier;
        m_poseConsumer = poseConsumer;

        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    }

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param time the time to wait
     */
    public PoseAnimator(Pose3d startPose, Supplier<Pose3d> endPoseSupplier, Consumer<Pose3d> poseConsumer, Time time) {
        this(startPose, endPoseSupplier, poseConsumer, time.in(Seconds));
    }

    public Pose3d getPose() {
        return m_startPose.interpolate(m_endPoseSupplier.get(), m_timer.get() / m_duration);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_poseConsumer.accept(getPose());
    }

    @Override
    public void execute() {
        m_poseConsumer.accept(getPose());
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("duration", () -> m_duration, null);
    }
}
