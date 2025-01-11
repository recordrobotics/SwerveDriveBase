package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Elevator extends KillableSubsystem implements ShuffleboardPublisher {

  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.Elevator.kP,
          Constants.Elevator.kI,
          Constants.Elevator.kD,
          constraints,
          Constants.Elevator.kDt);
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.Elevator.kS,
          Constants.Elevator.kG,
          Constants.Elevator.kV,
          Constants.Elevator.kA);

  public Elevator() {
    motorLeft = new TalonFX(Constants.Elevator.MOTOR_LEFT_ID);
    motorRight = new TalonFX(Constants.Elevator.MOTOR_RIGHT_ID);
  }

  private double getCurrentHeight() {
    // take sum of both motors to account for difference in heights for each motor
    return motorLeft.getPosition().getValueAsDouble() / Constants.Elevator.GEAR_RATIO
        + motorRight.getPosition().getValueAsDouble()
            / Constants.Elevator.GEAR_RATIO; // TODO: figure out how to convert rotations to height
  }

  private double getCurrentHeightDifference() {
    return motorLeft.getPosition().getValueAsDouble() / Constants.Elevator.GEAR_RATIO
        - /* NOTE: subtract the two to get differnce */ motorRight.getPosition().getValueAsDouble()
            / Constants.Elevator.GEAR_RATIO; // TODO: figure out how to convert rotations to height
  }

  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  @Override
  public void periodic() {
    // Run controller and update motor output
    double pidVal = controller.calculate(getCurrentHeight());
    double acceleration =
        (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double fwVal = feedforward.calculate(controller.getSetpoint().velocity, acceleration);
    motorLeft.setVoltage(pidVal + fwVal);
    motorRight.setVoltage(pidVal + fwVal);

    // TODO: replace this stuff to also include minimizing getCurrentHeightDifference() to 0

    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public void toggle(double heightMeters) {
    controller.setGoal(
        heightMeters * 2 /* double because current height is sum of both left+right */);
  }

  public void moveTo(ElevatorHeight height) {
    switch (height) {
      case OFF:
        kill();
        break;
      default:
        toggle(height.getHeight());
        break;
    }
  }

  @Override
  public void kill() {
    motorLeft.setVoltage(0);
    motorRight.setVoltage(0);
  }

  @Override
  public void close() {
    motorLeft.close();
    motorRight.close();
  }

  @Override
  public void setupShuffleboard() {
    ShuffleboardUI.Test.addSlider("Elevator Target", controller.getGoal().position, -90, 90)
        .subscribe(this::toggle);
  } // TODO add test slider for target
}
