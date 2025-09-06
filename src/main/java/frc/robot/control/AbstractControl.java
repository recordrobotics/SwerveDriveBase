package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.modifiers.DrivetrainControl;

public interface AbstractControl {

    void update();

    // Movement
    DrivetrainControl getDrivetrainControl();

    Transform2d getRawDriverInput();

    // Misc
    boolean isPoseResetTriggered();

    boolean isKillTriggered();

    void vibrate(RumbleType type, double value);

    // Orient XY
    static Pair<Double, Double> orientXY(Pair<Double, Double> input) {
        double inputX = input.getFirst();
        double inputY = input.getSecond();

        switch (DashboardUI.Overview.getDriverOrientation()) {
            case X_AXIS_TOWARDS_TRIGGER:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return new Pair<>(-inputY, -inputX);
                else return new Pair<>(inputY, inputX);
            case Y_AXIS_TOWARDS_TRIGGER:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return new Pair<>(inputX, -inputY);
                else return new Pair<>(-inputX, inputY);
            case X_AXIS_INVERTED_TOWARDS_TRIGGER:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return new Pair<>(inputY, inputX);
                else return new Pair<>(-inputY, -inputX);
            default:
                return new Pair<>(0.0, 0.0);
        }
    }

    // Orient Angle
    static Rotation2d orientAngle(Rotation2d angle) {
        switch (DashboardUI.Overview.getDriverOrientation()) {
            case X_AXIS_TOWARDS_TRIGGER:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
                    return new Rotation2d(angle.getRadians() - Math.PI / 2);
                else return new Rotation2d(angle.getRadians() + Math.PI / 2);
            case Y_AXIS_TOWARDS_TRIGGER:
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return angle;
                else return new Rotation2d(angle.getRadians() + Math.PI);
            default:
                return angle;
        }
    }
}
