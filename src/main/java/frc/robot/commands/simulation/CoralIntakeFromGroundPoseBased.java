package frc.robot.commands.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.HashSet;
import java.util.function.Supplier;

/**
 * Simulates coral intake from ground based on a series of coral poses (when you drive up to them)
 */
public class CoralIntakeFromGroundPoseBased extends SequentialCommandGroup
    implements SimulationCommand {

  private NamedCoral closestCoral;

  public CoralIntakeFromGroundPoseBased(Supplier<NamedCoral[]> coralPosesSupplier) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        // wait until we are close to a coral
        new WaitUntilCommand(
            () -> {
              Translation3d pose =
                  RobotContainer.model.coralIntake.getCoralTargetPose().getTranslation();

              NamedCoral[] coralPoses = coralPosesSupplier.get();

              if (coralPoses.length == 0) return false;

              closestCoral = coralPoses[0];
              double closestCoralDistance =
                  closestCoral.pose.get().getTranslation().getDistance(pose);

              for (int i = 1; i < coralPoses.length; i++) {
                NamedCoral coral = coralPoses[i];
                double distance = coral.pose.get().getTranslation().getDistance(pose);
                if (distance < closestCoralDistance) {
                  closestCoral = coral;
                  closestCoralDistance = distance;
                }
              }

              return closestCoralDistance < 0.2;
            }),
        new DeferredCommand(
            () -> {
              Pose3d intake = RobotContainer.model.coralIntake.getCoralTargetPose();
              Pose3d coral = closestCoral.pose.get();

              double dt1 =
                  Math.abs(
                      intake.getRotation().getZ()
                          - MathUtil.angleModulus(coral.getRotation().getZ()));
              double dt2 =
                  Math.abs(
                      intake.getRotation().getZ()
                          - MathUtil.angleModulus(coral.getRotation().getZ() + Math.PI));

              if (dt2 < dt1) {
                closestCoral.pose =
                    () ->
                        new Pose3d(
                            coral.getTranslation(),
                            coral.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
              }

              return new PoseAnimator(
                  closestCoral.pose.get(),
                  () -> RobotContainer.model.coralIntake.getCoralTargetPose(),
                  p -> closestCoral.pose = () -> p,
                  0.2);
            },
            new HashSet<>()),
        // mark coral for use for the elevator
        new InstantCommand(
            () -> {
              closestCoral.name = "CoralIntakeToElevator/Coral";
              closestCoral.pose = () -> RobotContainer.model.coralIntake.getCoralTargetPose();
            }),
        // set has coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralIntake.getSimIO().setCoralDetectorSim(false);
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
