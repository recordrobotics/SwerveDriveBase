package frc.robot.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.control.AbstractControl;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class OverviewLayout extends AbstractLayout {

  public enum DriverOrientation {
    XAxisTowardsTrigger("Competition"),
    YAxisTowardsTrigger("Y Axis"),
    XAxisInvertTowardsTrigger("Couch Drive");

    public final String display_name;

    DriverOrientation(String orientation) {
      display_name = orientation;
    }
  }

  private static LoggedDashboardChooser<DriverOrientation> driverOrientation =
      new LoggedDashboardChooser<DriverOrientation>("Driver Orientation");
  private static LoggedDashboardChooser<AbstractControl> driveMode =
      new LoggedDashboardChooser<AbstractControl>("Drive Mode");
  private static AbstractControl _defaultControl;

  // private Supplier<Boolean> poseCertainValue = () -> false;
  private Supplier<Boolean> navSensorValue = () -> false;

  private Supplier<Integer> tagNumLeftValue = () -> 0;
  private Supplier<Double> confidenceLeftValue = () -> 0.0;
  private Supplier<Boolean> hasVisionLeftValue = () -> false;
  private Supplier<Boolean> limelightConnectedLeftValue = () -> false;

  private Supplier<Integer> tagNumCenterValue = () -> 0;
  private Supplier<Double> confidenceCenterValue = () -> 0.0;
  private Supplier<Boolean> hasVisionCenterValue = () -> false;
  private Supplier<Boolean> limelightConnectedCenterValue = () -> false;

  private static final Map<Integer, TuningData> shooterSpeedData = new HashMap<>();

  public OverviewLayout() {
    addValueSendable("Nav Sensor", () -> navSensorValue.get(), "boolean");

    addValueSendable("Left/Tag Count", () -> tagNumLeftValue.get(), "int");

    addValueSendable("Left/Confidence", () -> confidenceLeftValue.get(), "double");

    addValueSendable("Left/Has Vision", () -> hasVisionLeftValue.get(), "boolean");

    addValueSendable(
        "Left/Limelight Connected", () -> limelightConnectedLeftValue.get(), "boolean");

    addValueSendable("Center/Tag Count", () -> tagNumCenterValue.get(), "int");

    addValueSendable("Center/Confidence", () -> confidenceCenterValue.get(), "double");

    addValueSendable("Center/Has Vision", () -> hasVisionCenterValue.get(), "boolean");

    addValueSendable(
        "Center/Limelight Connected", () -> limelightConnectedCenterValue.get(), "boolean");
  }

  /**
   * Initializes the control object
   *
   * @param defaultControl the first term will always be the default control object
   * @param controls any other control objects you want to initialize
   */
  public void addControls(AbstractControl defaultControl, AbstractControl... controls) {
    _defaultControl = defaultControl;

    // Sets up drive mode options
    for (AbstractControl abstractControl : controls) {
      driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
    }
    driveMode.addDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

    EnumSet.allOf(DriverOrientation.class)
        .forEach(v -> driverOrientation.addOption(v.display_name, v));
    driverOrientation.addDefaultOption(
        DriverOrientation.XAxisTowardsTrigger.display_name, DriverOrientation.XAxisTowardsTrigger);
  }

  public void setNavSensor(Supplier<Boolean> navSensor) {
    navSensorValue = navSensor;
  }

  public void setTagNumLeft(Supplier<Integer> tagNum) {
    tagNumLeftValue = tagNum;
  }

  public void setConfidenceLeft(Supplier<Double> confidence) {
    confidenceLeftValue = confidence;
  }

  public void setHasVisionLeft(Supplier<Boolean> hasVision) {
    hasVisionLeftValue = hasVision;
  }

  public void setLimelightConnectedLeft(Supplier<Boolean> limelightConnected) {
    limelightConnectedLeftValue = limelightConnected;
  }

  public void setTagNumCenter(Supplier<Integer> tagNum) {
    tagNumCenterValue = tagNum;
  }

  public void setConfidenceCenter(Supplier<Double> confidence) {
    confidenceCenterValue = confidence;
  }

  public void setHasVisionCenter(Supplier<Boolean> hasVision) {
    hasVisionCenterValue = hasVision;
  }

  public void setLimelightConnectedCenter(Supplier<Boolean> limelightConnected) {
    limelightConnectedCenterValue = limelightConnected;
  }

  public void putShooterSpeedData(int id, double current, double target) {
    shooterSpeedData.put(id, new TuningData(current, target));
  }

  @Override
  public void switchTo() {
    Elastic.selectTab("Overview");
  }

  @Override
  protected NetworkTable getNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("/SmartDashboard/Overview");
  }

  public DriverOrientation getDriverOrientation() {
    return driverOrientation.get();
  }

  public AbstractControl getControl() {
    if (driveMode.get() == null) return _defaultControl;
    return driveMode.get();
  }
}
