package frc.robot.commands.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ReefScoringPose;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.HashSet;

public class CoralIntakeToReef extends SequentialCommandGroup implements SimulationCommand {

  private NamedCoral coral;
  private ReefScoringPose reefPose;

  public CoralIntakeToReef() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(
            () -> coral = RobotContainer.model.getCoral("CoralIntakeToElevator/Coral")),
        // move coral to reef
        new DeferredCommand(
            () -> {
              if (coral == null) return new InstantCommand(() -> {});

              reefPose = ReefScoringPose.closestTo(coral.pose.get(), 0.5);
              if (reefPose == null) {
                // invalid shooting position, throw away coral
                return new InstantCommand(
                    () -> {
                      if (coral == null) return;
                      RobotContainer.model.removeCoral(coral);
                      coral = null;
                    });
              }

              Pose3d coralPose = coral.pose.get();

              double dt1 =
                  Math.abs(
                      reefPose.getPose().getRotation().getY()
                          - MathUtil.angleModulus(coralPose.getRotation().getY()));
              double dt2 =
                  Math.abs(
                      reefPose.getPose().getRotation().getY()
                          - MathUtil.angleModulus(coralPose.getRotation().getY() + Math.PI));

              if (dt2 < dt1) {
                coral.pose =
                    () ->
                        new Pose3d(
                            coralPose.getTranslation(),
                            coralPose.getRotation().plus(new Rotation3d(0, Math.PI, 0)));
              }

              return new PoseAnimator(
                  coral != null ? coral.pose.get() : new Pose3d(),
                  () -> reefPose.getPose(),
                  p -> {
                    if (coral != null) coral.pose = () -> p;
                  },
                  0.2);
            },
            new HashSet<Subsystem>()),
        new InstantCommand(
            () -> {
              if (coral != null) coral.name = "Reef/" + reefPose.name() + "/Coral";
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
