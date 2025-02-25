package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedAlgae;
import java.util.HashSet;

public class AlgaeGrabberFromGround extends SequentialCommandGroup implements SimulationCommand {

  private Pose3d algaePose;
  private NamedAlgae algae;

  public AlgaeGrabberFromGround(double secondsUntilAlgaeAcquired) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(
            () -> {
              Pose3d robotOrigin = new Pose3d();
              if (RobotContainer.poseTracker != null)
                robotOrigin = new Pose3d(RobotContainer.poseTracker.getEstimatedPosition());
              algaePose =
                  robotOrigin.transformBy(new Transform3d(0.7, 0, 0.2, new Rotation3d(0, 0, 0)));

              algae = new NamedAlgae("AlgaeGrabberFromGround/Algae", algaePose);

              RobotContainer.model.addAlgae(algae);
            }),
        new DeferredCommand(
            () ->
                new PoseAnimator(
                    algaePose,
                    () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseBottom(),
                    p -> algae.pose = () -> p,
                    secondsUntilAlgaeAcquired),
            new HashSet<>()),
        // set has algae (NC)
        new InstantCommand(
            () -> {
              try {
                algae.name = "AlgaeGrabber/Algae";
                algae.pose =
                    () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseBottom();
                RobotContainer.algaeGrabber.getSimIO().setAlgaeDetectorSim(false);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }));
  }

  @Override
  public Command simulateFor(Command command) {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      return command;
    } else {
      return command.alongWith(this);
    }
  }
}
