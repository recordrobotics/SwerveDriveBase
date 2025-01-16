package frc.robot.shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldStartingLocation;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousLayout extends AbstractLayout {

  private static final Field2d field = new Field2d();
  private static LoggedDashboardChooser<Command> autoChooser;
  private static LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
      new LoggedDashboardChooser<>("Starting Location");

  private static final Map<Integer, TuningData> velocityGraphData = new HashMap<>();

  public AutonomousLayout() {
    getTab().add(field).withWidget(BuiltInWidgets.kField).withSize(6, 3).withPosition(0, 0);

    EnumSet.allOf(FieldStartingLocation.class)
        .forEach(v -> fieldStartingLocationChooser.addOption(v.name(), v));
    fieldStartingLocationChooser.addDefaultOption(
        FieldStartingLocation.AutoStart.name(), FieldStartingLocation.AutoStart);

    // Creates the UI for starting location
    getTab()
        .add("Starting Location", fieldStartingLocationChooser)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withSize(6, 1)
        .withPosition(0, 3);
    getTab()
        .addDoubleArray("Velocity", () -> TuningData.MapToArray(velocityGraphData))
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(6, 2)
        .withSize(4, 3);
  }

  public void setupAutoChooser() {
    autoChooser = new LoggedDashboardChooser<>("Auto Code", AutoBuilder.buildAutoChooser());

    // Creates the UI for auto routines
    getTab()
        .add("Auto Code", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(3, 1)
        .withPosition(6, 1);
  }

  public void setRobotPose(Pose2d pose) {
    field.setRobotPose(pose);
  }

  public void setVisionPose(Pose2d pose) {
    field.getObject("Vision").setPose(pose);
    ;
  }

  public void putSwerveVelocityData(int id, double current, double target) {
    velocityGraphData.put(id, new TuningData(current, target));
  }

  @Override
  public ShuffleboardTab getTab() {
    return Shuffleboard.getTab("Autonomous");
  }

  public Command getAutoChooser() {
    return autoChooser.get();
  }

  public FieldStartingLocation getStartingLocation() {
    if (fieldStartingLocationChooser.get() == null) return FieldStartingLocation.AutoStart;
    return fieldStartingLocationChooser.get();
  }
}
