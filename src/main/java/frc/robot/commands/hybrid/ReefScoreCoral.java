package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldPosition;
import frc.robot.Constants.ReefCoralPosition;
import frc.robot.commands.GroundAlgaeScore;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class ReefScoreCoral extends SequentialCommandGroup {

  // Load the path we want to pathfind to and follow
  private PathPlannerPath path;

  private Alert pathNotFoundAlert = new Alert("Path Not Found: ProcessorScore", AlertType.kError);

  public ReefScoreCoral(ReefCoralPosition reefCoralPosition) {
    try {
      path = PathPlannerPath.fromPathFile("ProcessorScore");
    } catch (Exception e) {
      pathNotFoundAlert.set(true);
    }

    addCommands(
        AutoBuilder.pathfindThenFollowPath(path, Constants.HybridConstants.constraints),
        new GroundAlgaeScore());
  }
}
