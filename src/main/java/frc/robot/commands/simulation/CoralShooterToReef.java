package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
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

public class CoralShooterToReef extends SequentialCommandGroup implements SimulationCommand {

  private NamedCoral coral;
  private ReefScoringPose reefPose;

  public CoralShooterToReef() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(() -> coral = RobotContainer.model.getCoral("CoralShooter/Coral")),
        // set does not have coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralShooter.getSimIO().setCoralDetectorSim(true);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }),
        // move coral to elevator
        new DeferredCommand(
            () -> {
              reefPose = ReefScoringPose.closestTo(coral.pose.get(), 0.5);
              if (reefPose == null) {
                // invalid shooting position, throw away coral
                return new InstantCommand(
                    () -> {
                      RobotContainer.model.removeCoral(coral);
                      coral = null;
                    });
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
