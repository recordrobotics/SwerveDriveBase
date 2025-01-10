package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator extends KillableSubsystem implements ShuffleboardPublisher {

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

  private final Encoder encoder = new Encoder(1, 2); // TODO ports go somewhere else
  private final TalonFX motor = new TalonFX(-1); // TODO what port

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
    encoder.setDistancePerPulse(
        1.0 / 360.0 * 2.0 * Math.PI
            * 1.5); // TODO should be a constant and also what math is this bro
  }

  @Override
  public void periodic() {
    // Run controller and update motor output
    motor.setVoltage(
        controller.calculate(encoder.getDistance())
            + feedforward.calculate(controller.getSetpoint().velocity));
  }

  public void toggle(double setpoint) {
    controller.setGoal(setpoint);
  }

  public void toggle(ElevatorStates state) {
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
        motor.setVoltage(0);
        break;
    }
  }

  @Override
  public void kill() {
    motor.set(0); // TODO set or setVoltage
  }

  @Override
  public void close() {
    motor.close();
  }

  @Override
  public void setupShuffleboard() {} // TODO what shuffleboard things do we want
}
