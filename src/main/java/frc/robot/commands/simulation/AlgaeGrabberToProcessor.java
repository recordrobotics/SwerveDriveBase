package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ProcessorPose;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedAlgae;
import java.util.HashSet;

public class AlgaeGrabberToProcessor extends SequentialCommandGroup implements SimulationCommand {

  private NamedAlgae algae;
  private ProcessorPose processorPose;

  public AlgaeGrabberToProcessor() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(() -> algae = RobotContainer.model.getAlgae("AlgaeGrabber/Algae")),
        new InstantCommand(
            () -> {
              try {
                RobotContainer.algaeGrabber.getSimIO().setHasAlgae(false);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }),
        // move coral to elevator
        new DeferredCommand(
            () -> {
              if (algae == null) return new InstantCommand(() -> {});

              processorPose = ProcessorPose.closestTo(algae.pose.get(), 0.5);
              if (processorPose == null) {
                // invalid scoring position, throw away algae
                return new InstantCommand(
                    () -> {
                      if (algae == null) return;
                      RobotContainer.model.removeAlgae(algae);
                      algae = null;
                    });
              }

              return new PoseAnimator(
                      algae != null ? algae.pose.get() : new Pose3d(),
                      () -> processorPose.getPose(),
                      p -> {
                        if (algae != null) algae.pose = () -> p;
                      },
                      0.2)
                  .andThen(
                      () -> {
                        new PoseAnimator(
                                processorPose.getPose(),
                                () ->
                                    processorPose
                                        .getPose()
                                        .transformBy(
                                            new Transform3d(1.5, 0, -0.3, new Rotation3d())),
                                p -> {
                                  if (algae != null) algae.pose = () -> p;
                                },
                                0.6)
                            .andThen(
                                () -> {
                                  if (algae != null) RobotContainer.model.removeAlgae(algae);
                                })
                            .schedule();
                      });
            },
            new HashSet<Subsystem>()),
        new InstantCommand(
            () -> {
              if (algae != null) algae.name = "Processor/" + processorPose.name() + "/Algae";
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
