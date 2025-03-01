package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMove;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.TriggerProcessor.TriggerDistance;
import java.util.Set;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class HybridScoreCoral extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public static DeferredCommand deferred(ElevatorHeight reefCoralHeight) {
    return new DeferredCommand(
        () -> new HybridScoreCoral(reefCoralHeight), Set.of(RobotContainer.coralShooter));
  }

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

    Command stateVisualizerLightsCommand =
        RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.hybridPattern);
    Command elevatorLightsCommand =
        RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);

    addCommands(
        new ConditionalCommand( // cancel if no coral
            new InstantCommand(() -> this.cancel()),
            new InstantCommand(() -> {}),
            () -> !RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(() -> stateVisualizerLightsCommand.schedule()),
        AutoBuilder.pathfindToPose(shortestPose, Constants.HybridConstants.constraints).asProxy(),
        new InstantCommand(() -> elevatorLightsCommand.schedule()),
        AutoBuilder.followPath(shortestPath).asProxy().alongWith(new ElevatorMove(reefCoralHeight)),
        new InstantCommand(stateVisualizerLightsCommand::cancel),
        new InstantCommand(elevatorLightsCommand::cancel));
    // new ElevatorMoveThenCoralShoot(reefCoralHeight));
  }
}
