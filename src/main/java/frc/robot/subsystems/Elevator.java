package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.HashMap;
import java.util.Map;

public class Elevator extends KillableSubsystem implements ShuffleboardPublisher {
  private static final Map<ElevatorStates, Double> stateToHieght = new HashMap<>();

  // TODO move these to Constants.java
  private static double kDt = 0.02;
  private static double kMaxVelocity = 1.75;
  private static double kMaxAcceleration = 0.75;
  private static double kP = 1.3;
  private static double kI = 0.0;
  private static double kD = 0.7;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  private final TalonFX motor;
  private final TalonFX motor2;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(kP, kI, kD, constraints, kDt);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV);

  public enum ElevatorStates {
    INTAKE,
    L1,
    L2,
    L3,
    L4,
    OFF
  }

  public Elevator() {
    motor = new TalonFX(-1); // TODO what port and ports go somewhere else
    motor2 = new TalonFX(-1); // TODO what port and ports go somewhere else
  }

  private double getCurrentHieght() {
    return motor.getPosition().getValueAsDouble(); // TODO might work, should check
  }

  private void setMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
    motor2.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // Run controller and update motor output
    setMotorVoltage(
        controller.calculate(getCurrentHieght())
            + feedforward.calculate(controller.getSetpoint().velocity));
  }

  public void toggle(double setpoint) {
    controller.setGoal(setpoint);
  }

  public void moveTo(ElevatorStates state) {
    switch (state) {
      case INTAKE:
        toggle(0);
        break;
      case L1:
        toggle(1);
        break;
      case L2:
        toggle(2);
        break;
      case L3:
        toggle(3);
        break;
      case L4:
        toggle(4);
        break;
      case OFF:
      default:
        kill();
        break;
    }
  }

  @Override
  public void kill() {
    setMotorVoltage(0);
  }

  @Override
  public void close() {
    motor.close();
  }

  @Override
  public void setupShuffleboard() {} // TODO add test slider for target
}
