package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMoveThenAlgaeGrab;
import frc.robot.commands.SuccessfulCompletion;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class HybridRemoveAlgae extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public HybridRemoveAlgae() {
    addCommands(
        new InstantCommand(
            () ->
                new SequentialCommandGroup(
                        new InstantCommand(
                            () ->
                                RobotContainer.lights.patterns.put(
                                    LightSegments.HYBRID_STATES,
                                    () -> Constants.Lights.removeAlgaePattern)))
                    .schedule()));
    try {
      paths =
          new PathPlannerPath[] {
            PathPlannerPath.fromPathFile("Approach Algae AB"),
            PathPlannerPath.fromPathFile("Approach Algae CD"),
            PathPlannerPath.fromPathFile("Approach Algae EF"),
            PathPlannerPath.fromPathFile("Approach Algae GH"),
            PathPlannerPath.fromPathFile("Approach Algae IJ"),
            PathPlannerPath.fromPathFile("Approach Algae KL")
          };
    } catch (Exception e) {
      pathNotFoundAlert.setText("Path Not Found! HybridRemoveAlgae");
      pathNotFoundAlert.set(true);
      return; // TODO is this the right way to exit a command? (maybie ```end(true);``` ?)
    }

    // Gets path closest to the robot
    PathPlannerPath shortestPath = paths[0];
    double lowestDistance = 1000;
    Translation2d swerve_translation =
        RobotContainer.poseTracker.getEstimatedPosition().getTranslation();
    for (PathPlannerPath path : paths) {
      Translation2d path_translation = path.getStartingHolonomicPose().get().getTranslation();
      double distance = swerve_translation.getDistance(path_translation);
      if (distance < lowestDistance) {
        lowestDistance = distance;
        shortestPath = path;
      }
    }

    ElevatorHeight algaeHeight =
        Constants.HybridConstants.isAlgaeInHighPosition.get(shortestPath.name)
            ? ElevatorHeight.HIGH_ALGAE
            : ElevatorHeight.LOW_ALGAE;

    addCommands(
        AutoBuilder.pathfindToPose(
            shortestPath.getStartingHolonomicPose().get(), Constants.HybridConstants.constraints),
        new InstantCommand(() -> RobotContainer.elevator.moveTo(algaeHeight)),
        AutoBuilder.followPath(shortestPath),
        new ElevatorMoveThenAlgaeGrab(algaeHeight),
        new SuccessfulCompletion(true, false, true, false, true));
  }
}
