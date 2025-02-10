package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;

/**
 * Simulates coral intake from ground based on a series of coral poses (when you drive up to them)
 */
public class CoralIntakeFromGroundPoseBased extends SequentialCommandGroup
    implements SimulationCommand {

  private NamedCoral closestCoral;

  public CoralIntakeFromGroundPoseBased(NamedCoral[] coralPoses) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;
    if (coralPoses.length == 0) return;

    addCommands(
        // wait until we are close to a coral
        new WaitUntilCommand(
            () -> {
              Translation2d pose =
                  RobotContainer.poseTracker.getEstimatedPosition().getTranslation();

              closestCoral = coralPoses[0];
              double closestCoralDistance =
                  closestCoral.pose.getTranslation().toTranslation2d().getDistance(pose);

              for (int i = 1; i < coralPoses.length; i++) {
                NamedCoral coral = coralPoses[i];
                double distance = coral.pose.getTranslation().toTranslation2d().getDistance(pose);
                if (distance < closestCoralDistance) {
                  closestCoral = coral;
                  closestCoralDistance = distance;
                }
              }

              return closestCoralDistance < 0.02;
            }),
        // TODO: move coral to intake
        // mark coral for use for the elevator
        new InstantCommand(() -> closestCoral.name = "CoralIntakeToElevator/Coral"),
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
