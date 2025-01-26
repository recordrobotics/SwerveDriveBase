package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.Constants.ReefCoralPosition;
import frc.robot.commands.ElevatorMoveThenCoralShoot;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class ReefScoreCoral extends SequentialCommandGroup {

  private PathPlannerPath path;

  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);

  public ReefScoreCoral(ReefCoralPosition reefCoralPosition, ElevatorHeight reefCoralHeight) {
    try {
      path = PathPlannerPath.fromPathFile(reefCoralPosition.getApproachPathName());
    } catch (Exception e) {
      pathNotFoundAlert.setText("Path Not Found: " + reefCoralPosition.getApproachPathName());
      pathNotFoundAlert.set(true);
    }

    addCommands(
        AutoBuilder.pathfindThenFollowPath(path, Constants.HybridConstants.constraints),
        new ElevatorMoveThenCoralShoot(reefCoralHeight));
  }
}
