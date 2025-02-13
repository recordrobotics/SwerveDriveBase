package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.HashSet;

public class CoralIntakeFromSourceSim extends SequentialCommandGroup implements SimulationCommand {

  private Pose3d coralPose;
  private NamedCoral coral;

  public CoralIntakeFromSourceSim() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(
            () -> {
              Pose2d robotOrigin = new Pose2d();
              if (RobotContainer.poseTracker != null)
                robotOrigin = RobotContainer.poseTracker.getEstimatedPosition();

              Pose3d closestSource =
                  new Pose3d(Constants.FieldConstants.closestSourceTo(robotOrigin));

              coralPose =
                  closestSource.transformBy(
                      new Transform3d(0, 0.834669, 1.3, new Rotation3d(0, 0, 0)));

              coral = new NamedCoral("CoralIntakeFromSource/Coral", coralPose);

              RobotContainer.model.addCoral(coral);
            }),
        new DeferredCommand(
            () ->
                new PoseAnimator(
                    coralPose,
                    () -> RobotContainer.model.coralIntake.getCoralTargetPose(),
                    p -> coral.pose = () -> p,
                    0.2),
            new HashSet<>()),
        // set has elevator coral (NC)
        new InstantCommand(
            () -> {
              coral.name = "CoralShooter/Coral";
              coral.pose = () -> RobotContainer.model.elevator.getCoralShooterTargetPose();
              try {
                RobotContainer.coralShooter.getSimIO().setCoralDetectorSim(false);
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
