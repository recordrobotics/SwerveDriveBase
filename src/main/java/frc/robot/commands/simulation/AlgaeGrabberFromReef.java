package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ReefAlgaePose;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedAlgae;
import java.util.HashSet;

public class AlgaeGrabberFromReef extends SequentialCommandGroup implements SimulationCommand {

  private Pose3d algaePose;
  private NamedAlgae algae;

  public AlgaeGrabberFromReef() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new WaitUntilCommand(
            () -> {
              ReefAlgaePose closestAlgae =
                  ReefAlgaePose.closestTo(
                      RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseTop(), 1.2);

              if (closestAlgae == null) return false;

              Pose3d closestAlgaePose = closestAlgae.getPose();

              Pose3d algaePose =
                  closestAlgaePose.transformBy(new Transform3d(0, 0, 0, new Rotation3d()));

              Pose3d grabber = RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseTop();
              return grabber.getTranslation().getDistance(algaePose.getTranslation()) < 0.4;
            }),
        new InstantCommand(
            () -> {
              ReefAlgaePose closestAlgae =
                  ReefAlgaePose.closestTo(
                      RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseTop(), 1.2);

              if (closestAlgae == null) return;

              algaePose = closestAlgae.getPose();

              algae = new NamedAlgae("AlgaeGrabber/Algae", algaePose);

              RobotContainer.model.addAlgae(algae);
            }),
        new DeferredCommand(
            () -> {
              return new PoseAnimator(
                  algaePose,
                  () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseTop(),
                  p -> algae.pose = () -> p,
                  0.2);
            },
            new HashSet<>()),
        new InstantCommand(
            () -> {
              try {
                algae.pose = () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseTop();
                RobotContainer.elevatorHead.getSimIO().setHasAlgae(true);
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
