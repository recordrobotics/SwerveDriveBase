package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedAlgae;
import java.util.HashSet;
import java.util.function.Supplier;

public class AlgaeGrabberFromGroundPoseBased extends SequentialCommandGroup
    implements SimulationCommand {

  private NamedAlgae closestAlgae;

  public AlgaeGrabberFromGroundPoseBased(Supplier<NamedAlgae[]> algaePosesSupplier) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        // wait until we are close to an algae
        new WaitUntilCommand(
            () -> {
              Translation3d pose =
                  RobotContainer.model.coralIntake.getCoralTargetPose().getTranslation();

              NamedAlgae[] algaePoses = algaePosesSupplier.get();

              if (algaePoses.length == 0) return false;

              closestAlgae = algaePoses[0];
              double closestAlgaeDistance =
                  closestAlgae.pose.get().getTranslation().getDistance(pose);

              for (int i = 1; i < algaePoses.length; i++) {
                NamedAlgae algae = algaePoses[i];
                double distance = algae.pose.get().getTranslation().getDistance(pose);
                if (distance < closestAlgaeDistance) {
                  closestAlgae = algae;
                  closestAlgaeDistance = distance;
                }
              }

              return closestAlgaeDistance < 0.4;
            }),
        new DeferredCommand(
            () ->
                new PoseAnimator(
                    closestAlgae.pose.get(),
                    () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseBottom(),
                    p -> closestAlgae.pose = () -> p,
                    0.2),
            new HashSet<>()),
        new InstantCommand(
            () -> {
              try {
                closestAlgae.name = "AlgaeGrabber/Algae";
                closestAlgae.pose =
                    () -> RobotContainer.model.elevatorArm.getAlgaeGrabberTargetPoseBottom();
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
