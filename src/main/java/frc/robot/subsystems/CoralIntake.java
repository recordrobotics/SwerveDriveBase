// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.KillableSubsystem;

public class CoralIntake extends KillableSubsystem {
  private final SparkMax motor;
  private final SparkMax servo;
  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final ProfiledPIDController servoPID =
      new ProfiledPIDController(
          Constants.CoralIntake.sP,
          Constants.CoralIntake.sI,
          Constants.CoralIntake.sD,
          new Constraints(
              Constants.CoralIntake.MAX_SERVO_VELOCITY,
              Constants.CoralIntake.MAX_SERVO_ACCELERATION));
  private final PIDController pid =
      new PIDController(
          Constants.CoralShooter.kP, Constants.CoralShooter.kI, Constants.CoralShooter.kD);
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Constants.CoralShooter.kS, Constants.CoralShooter.kV);

  public CoralIntake() {
    motor = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    servo = new SparkMax(RobotMap.CoralShooter.SERVO_ID, MotorType.kBrushless);
    toggle(CoralIntakeStates.OFF); // initialize as off
    ShuffleboardUI.Test.addSlider("Coral Intake", motor.get(), -1, 1).subscribe(motor::set);
    ShuffleboardUI.Test.addSlider("Coral Intake Pos", motor.getEncoder().getPosition(), -1, 1)
        .subscribe(this::toggleServo);
  }

  public enum CoralIntakeStates {
    REVERSE,
    INTAKE,
    OFF;
  }

  public boolean hasCoral() {
    return coralDetector.get();
  }

  public enum IntakeServoStates {
    UP,
    DOWN,
    OFF;
  }

  public double getWheelVelocity() {
    return motor.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  public void toggleServo(double pos) {
    servoPID.setGoal(pos);
  }

  public boolean atGoal() {
    return servoPID.atGoal();
  }

  public void toggleServo(IntakeServoStates state) {
    switch (state) {
      case UP:
        toggleServo(Constants.CoralIntake.SERVO_UP);
        break;
      case DOWN:
        toggleServo(Constants.CoralIntake.SERVO_DOWN);
        break;
      case OFF:
      default:
        servo.setVoltage(0);
        break;
    }
  }

  public boolean servoAtGoal() {
    return servoPID.atSetpoint();
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralIntakeStates state) {
    switch (state) {
      case REVERSE:
        toggle(Constants.CoralIntake.REVERSE_SPEED);
        break;
      case INTAKE:
        toggle(Constants.CoralIntake.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double pidOutputServo = servoPID.calculate(servo.getEncoder().getPosition());
    double feedforwardOutput = feedForward.calculate(pid.getSetpoint());
    motor.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    servo.setVoltage(pidOutputServo);
  }

  @Override
  public void kill() {
    toggle(CoralIntakeStates.OFF);
    motor.setVoltage(0);
    servo.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
    servo.close();
  }
}
