package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMoveThenCoralShoot;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class ReefScoreCoral extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public ReefScoreCoral(ElevatorHeight reefCoralHeight) {
    try {
      paths =
          new PathPlannerPath[] {
            PathPlannerPath.fromPathFile("Approach Coral A"),
            PathPlannerPath.fromPathFile("Approach Coral B"),
            PathPlannerPath.fromPathFile("Approach Coral C"),
            PathPlannerPath.fromPathFile("Approach Coral D"),
            PathPlannerPath.fromPathFile("Approach Coral E"),
            PathPlannerPath.fromPathFile("Approach Coral F"),
            PathPlannerPath.fromPathFile("Approach Coral G"),
            PathPlannerPath.fromPathFile("Approach Coral H"),
            PathPlannerPath.fromPathFile("Approach Coral I"),
            PathPlannerPath.fromPathFile("Approach Coral J"),
            PathPlannerPath.fromPathFile("Approach Coral K"),
            PathPlannerPath.fromPathFile("Approach Coral L"),
          };
    } catch (Exception e) {
      pathNotFoundAlert.setText("Path Not Found! ReefScoreCoral");
      pathNotFoundAlert.set(true);
      return; // TODO is this the right way to exit a command? (maybie ```end(true);``` ?)
    }

    // Gets path closest to the robot
    PathPlannerPath shortestPath = paths[0];
    double lowestDistance = 1000;
    Translation2d swerve_translation =
        RobotContainer.poseTracker.getEstimatedPosition().getTranslation();
    for (PathPlannerPath path : paths) {
      Translation2d path_translation =
          path.getStartingHolonomicPose()
              .orElseGet(
                  () ->
                      new Pose2d(
                          new Translation2d(
                              999999,
                              999999), // Far away place that will never be chosen as closest
                          new Rotation2d()))
              .getTranslation();
      double distance = swerve_translation.getDistance(path_translation);
      if (distance < lowestDistance) {
        lowestDistance = distance;
        shortestPath = path;
      }
    }

    addCommands(
        AutoBuilder.pathfindThenFollowPath(shortestPath, Constants.HybridConstants.constraints),
        new ElevatorMoveThenCoralShoot(reefCoralHeight));
  }
}
