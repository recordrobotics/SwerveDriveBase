package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ElevatorMoveThenCoralShoot;
import frc.robot.commands.LightsCommand;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class HybridScoreCoral extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public HybridScoreCoral(ElevatorHeight reefCoralHeight) {
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
      cancel();
    }

    // Gets path closest to the robot
    PathPlannerPath shortestPath = paths[0];
    double lowestDistance = 1000;
    Translation2d swerve_translation =
        RobotContainer.poseTracker.getEstimatedPosition().getTranslation();
    for (PathPlannerPath path : paths) {
      Translation2d path_translation = path.getStartingHolonomicPose().get().getTranslation();
      if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
        path_translation = FlippingUtil.flipFieldPosition(path_translation);
      }

      double distance = swerve_translation.getDistance(path_translation);
      if (distance < lowestDistance) {
        lowestDistance = distance;
        shortestPath = path;
      }
    }

    Pose2d shortestPose = shortestPath.getStartingHolonomicPose().get();

    if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
      shortestPose = FlippingUtil.flipFieldPose(shortestPose);
    }

    addCommands(
        new ConditionalCommand( // cancel if no coral
            new InstantCommand(() -> this.cancel()),
            new InstantCommand(() -> {}),
            () -> !RobotContainer.coralShooter.hasCoral()),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.hybridPattern),
        AutoBuilder.pathfindToPose(shortestPose, Constants.HybridConstants.constraints),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.elevatorPattern),
        AutoBuilder.followPath(shortestPath).alongWith(new ElevatorMove(reefCoralHeight)),
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.OFF),
        new LightsCommand(LightSegments.ELEVATOR, Constants.Lights.OFF),
        new ElevatorMoveThenCoralShoot(reefCoralHeight));
  }
}
