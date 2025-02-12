package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.Random;

public class PlaceRandomGroundCoral extends Command {

  private static int counter = 0;
  private final Random random;

  public PlaceRandomGroundCoral() {
    random = new Random();
  }

  private Translation2d sampleRandomPosition(double x1, double y1, double x2, double y2) {
    double sourceDistance = 0.8;
    double reefDistance = 1.0;

    Translation2d t;
    do {
      t = new Translation2d(random.nextDouble(x1, x2), random.nextDouble(y1, y2));
    } while (t.getDistance(Constants.FieldConstants.SOURCE_1.getTranslation()) < sourceDistance
        || t.getDistance(Constants.FieldConstants.SOURCE_2.getTranslation()) < sourceDistance
        || t.getDistance(Constants.FieldConstants.SOURCE_12.getTranslation()) < sourceDistance
        || t.getDistance(Constants.FieldConstants.SOURCE_13.getTranslation()) < sourceDistance
        || t.getDistance(Constants.FieldConstants.TEAM_BLUE_REEF_CENTER) < reefDistance
        || t.getDistance(Constants.FieldConstants.TEAM_RED_REEF_CENTER) < reefDistance);

    return t;
  }

  @Override
  public void initialize() {

    Translation2d translation;

    if (RobotContainer.poseTracker.getEstimatedPosition().getTranslation().getX()
        < Constants.FieldConstants.FIELD_X_DIMENSION / 2) {
      // robot is on blue side
      translation =
          sampleRandomPosition(
              0,
              0,
              Constants.FieldConstants.FIELD_X_DIMENSION / 2,
              Constants.FieldConstants.FIELD_Y_DIMENSION);
    } else {
      // robot is on red side
      translation =
          sampleRandomPosition(
              Constants.FieldConstants.FIELD_X_DIMENSION / 2,
              0,
              Constants.FieldConstants.FIELD_X_DIMENSION,
              Constants.FieldConstants.FIELD_Y_DIMENSION);
    }

    double angle = random.nextDouble(Math.PI);
    NamedCoral coral =
        new NamedCoral(
            "GroundCoral/Random_" + counter,
            () ->
                new Pose3d(
                    translation.getX(), translation.getY(), 0.06, new Rotation3d(0, 0, angle)));
    counter++;
    RobotContainer.model.addCoral(coral);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
