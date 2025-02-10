package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.HashSet;

public class CoralIntakeToElevator extends SequentialCommandGroup implements SimulationCommand {

  private NamedCoral coral;

  public CoralIntakeToElevator() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(
            () -> coral = RobotContainer.model.getCoral("CoralIntakeToElevator/Coral")),
        // set does not have coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralIntake.getSimIO().setCoralDetectorSim(true);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }),
        // move coral to elevator
        new DeferredCommand(
            () ->
                new PoseAnimator(
                        coral
                            .pose
                            .get()
                            .relativeTo(
                                new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())),
                        () ->
                            RobotContainer.model
                                .elevator
                                .getCoralIntakeEjectPose()
                                .relativeTo(
                                    new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())),
                        p ->
                            coral.pose =
                                () ->
                                    new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())
                                        .plus(new Transform3d(p.getTranslation(), p.getRotation())),
                        0.1)
                    .andThen(
                        new PoseAnimator(
                            RobotContainer.model
                                .elevator
                                .getCoralIntakeEjectPose()
                                .relativeTo(
                                    new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())),
                            () ->
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeEjectFinalPose()
                                    .relativeTo(
                                        new Pose3d(
                                            RobotContainer.poseTracker.getEstimatedPosition())),
                            p ->
                                coral.pose =
                                    () ->
                                        new Pose3d(
                                                RobotContainer.poseTracker.getEstimatedPosition())
                                            .plus(
                                                new Transform3d(
                                                    p.getTranslation(), p.getRotation())),
                            0.1))
                    .andThen(
                        new PoseAnimator(
                            RobotContainer.model
                                .elevator
                                .getCoralIntakeEjectFinalPose()
                                .relativeTo(
                                    new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())),
                            () ->
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeChannelPose()
                                    .relativeTo(
                                        new Pose3d(
                                            RobotContainer.poseTracker.getEstimatedPosition())),
                            p ->
                                coral.pose =
                                    () ->
                                        new Pose3d(
                                                RobotContainer.poseTracker.getEstimatedPosition())
                                            .plus(
                                                new Transform3d(
                                                    p.getTranslation(), p.getRotation())),
                            0.15))
                    .andThen(
                        new PoseAnimator(
                            RobotContainer.model
                                .elevator
                                .getCoralIntakeChannelPose()
                                .relativeTo(
                                    new Pose3d(RobotContainer.poseTracker.getEstimatedPosition())),
                            () ->
                                RobotContainer.model
                                    .elevator
                                    .getCoralShooterTargetPose()
                                    .relativeTo(
                                        new Pose3d(
                                            RobotContainer.poseTracker.getEstimatedPosition())),
                            p ->
                                coral.pose =
                                    () ->
                                        new Pose3d(
                                                RobotContainer.poseTracker.getEstimatedPosition())
                                            .plus(
                                                new Transform3d(
                                                    p.getTranslation(), p.getRotation())),
                            0.4)),
            new HashSet<Subsystem>()),
        // set has elevator coral (NC)
        new InstantCommand(
            () -> {
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
