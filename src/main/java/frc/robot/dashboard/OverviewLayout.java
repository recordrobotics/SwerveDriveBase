package frc.robot.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.control.AbstractControl;
import frc.robot.utils.libraries.Elastic;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class OverviewLayout extends AbstractLayout {

    public enum DriverOrientation {
        X_AXIS_TOWARDS_TRIGGER("Competition"),
        Y_AXIS_TOWARDS_TRIGGER("Y Axis"),
        X_AXIS_INVERTED_TOWARDS_TRIGGER("Couch Drive");

        public final String displayName;

        DriverOrientation(String orientation) {
            displayName = orientation;
        }
    }

    private final Field2d field = new Field2d();

    private LoggedDashboardChooser<DriverOrientation> driverOrientation =
            new LoggedDashboardChooser<>("Driver Orientation");
    private LoggedDashboardChooser<AbstractControl> driveMode = new LoggedDashboardChooser<>("Drive Mode");
    private LoggedDashboardChooser<FieldStartingLocation> fieldStartingLocationChooser =
            new LoggedDashboardChooser<>("Starting Location");
    private AbstractControl defaultControl;
    private AbstractControl testControl;

    private Supplier<Boolean> navSensorValue = () -> false;

    public OverviewLayout() {
        buildSendable("Field", field);
        addValueSendable("Nav Sensor", () -> navSensorValue.get(), "boolean");

        EnumSet.allOf(FieldStartingLocation.class)
                .forEach(v -> fieldStartingLocationChooser.addOption(v.toString(), v));
        fieldStartingLocationChooser.addDefaultOption(
                FieldStartingLocation.DEFAULT.toString(), FieldStartingLocation.DEFAULT);

        SmartDashboard.putBoolean("Autonomous/ResetLocationButton", false);
        SmartDashboard.putBoolean("Autonomous/EncoderReset", false);
    }

    /**
     * Initializes the control object
     *
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public void addControls(AbstractControl defaultControl, AbstractControl... controls) {
        this.defaultControl = defaultControl;

        // Sets up drive mode options
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }
        driveMode.addDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

        EnumSet.allOf(DriverOrientation.class).forEach(v -> driverOrientation.addOption(v.displayName, v));
        driverOrientation.addDefaultOption(
                DriverOrientation.X_AXIS_TOWARDS_TRIGGER.displayName, DriverOrientation.X_AXIS_TOWARDS_TRIGGER);
    }

    public void setNavSensor(Supplier<Boolean> navSensor) {
        navSensorValue = navSensor;
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

    public void setTestControl(AbstractControl testControl) {
        if (Constants.RobotState.getMode() != Constants.RobotState.Mode.TEST) return;
        this.testControl = testControl;
    }

    public AbstractControl getControl() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST) {
            if (testControl == null) {
                throw new IllegalStateException("Test control is not set!");
            }
            return testControl;
        }

        if (driveMode.get() == null) return defaultControl;
        return driveMode.get();
    }

    public void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setVisionPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    public FieldStartingLocation getStartingLocation() {
        return fieldStartingLocationChooser.get() == null
                ? FieldStartingLocation.DEFAULT
                : fieldStartingLocationChooser.get();
    }

    public boolean isResetLocationPressed() {
        return SmartDashboard.getBoolean("Autonomous/ResetLocationButton", false);
    }

    public boolean isEncoderResetPressed() {
        return SmartDashboard.getBoolean("Autonomous/EncoderReset", false);
    }
}
