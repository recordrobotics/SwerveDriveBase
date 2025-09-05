package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

@SuppressWarnings({"java:S109"})
public class OnlyJoystick implements AbstractControl {

    private Joystick joystick;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    public OnlyJoystick(int joystickPort) {
        joystick = new Joystick(joystickPort);
    }

    @Override
    public void update() {
        Pair<Double, Double> xy = getXYOriented();

        double x = xy.getFirst() * getDirectionalSpeedLevel();
        double y = xy.getSecond() * getDirectionalSpeedLevel();

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
        acceleration = new Transform2d(
                        velocity.getTranslation()
                                .minus(lastVelocity.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);

        lastVelocity = velocity;
        lastAcceleration = acceleration;
    }

    @Override
    public DrivetrainControl getDrivetrainControl() {
        return DrivetrainControl.createFieldRelative(
                velocity,
                acceleration,
                jerk,
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
    }

    @Override
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXYRaw();
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
    }

    public Pair<Double, Double> getXYRaw() {
        double x = SimpleMath.applyThresholdAndSensitivity(
                joystick.getX(),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
        double y = SimpleMath.applyThresholdAndSensitivity(
                joystick.getY(),
                Constants.Control.JOYSTICK_Y_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        return SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
                Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        return SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.SPIN_SPEED_METER_LOW,
                Constants.Control.SPIN_SPEED_METER_HIGH);
    }

    @Override
    public boolean isPoseResetTriggered() {
        return joystick.getRawButtonPressed(5);
    }

    @Override
    public boolean isKillTriggered() {
        return joystick.getRawButton(4);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        /* no vibrate on the joystick  */
    }
}
