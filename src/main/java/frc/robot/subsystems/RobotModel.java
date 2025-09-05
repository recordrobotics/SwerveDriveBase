package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.field.FieldIntersection;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

/** Represents the physical model of the robot, including mechanisms and their positions */
public final class RobotModel extends ManagedSubsystemBase {

    public RobotModel() {
        periodicManaged();
    }

    @Override
    public void periodicManaged() {
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.DEBUG_SIM)) {
            FieldIntersection.logPolygons();
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    private Pose3d[] getCoralPositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            List<Pose3d> coralPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Coral");
            return coralPoses.toArray(new Pose3d[0]);
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    private Pose3d[] getAlgaePositions() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
        } else {
            return new Pose3d[0];
        }
    }

    @AutoLogLevel(level = Level.SIM)
    @SuppressWarnings("java:S2325") // rest of the getters are non-static
    public Pose2d getRobot() {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
            return RobotContainer.drivetrain.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
        } else {
            return Pose2d.kZero;
        }
    }
}
