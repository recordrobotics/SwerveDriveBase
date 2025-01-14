package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;

public class GroundAlgae extends KillableSubsystem implements ShuffleboardPublisher {
  private Spark motor = new Spark(RobotMap.GroundAlgae.MOTOR_ID);
  private static final double defaultSpeed = Constants.GroundAlgae.DEFAULT_SPEED;
  private GroundAlgaeStates groundAlgaeState = GroundAlgaeStates.OFF;

  public GroundAlgaeStates getGroundAlgaeState() {
    return groundAlgaeState;
  }

  public GroundAlgae() {
    toggle(GroundAlgaeStates.OFF);
  }

  public enum GroundAlgaeStates {
    IN,
    OUT,
    OFF;
  }

  public void toggle(GroundAlgaeStates state, double speed) {
    groundAlgaeState = state;
    switch (state) {
      case IN: // take in note
        motor.set(speed);
        break;
      case OUT: // push out note
        motor.set(-speed);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        motor.set(0);
        break;
    }
  }

  public void toggle(GroundAlgaeStates state) {
    toggle(state, defaultSpeed);
  }

  @Override
  public void kill() {
    toggle(GroundAlgaeStates.OFF);
  }

  /** frees up all hardware allocations */
  @Override
  public void close() {
    motor.close();
  }

  @Override
  public void setupShuffleboard() {
    // TODO do we need other shuffleboard stuff
    ShuffleboardUI.Test.addMotor("GroundAlgae", motor);
  }
}
