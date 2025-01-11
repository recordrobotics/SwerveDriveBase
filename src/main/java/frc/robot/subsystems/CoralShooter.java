// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.RobotMap;

public class CoralShooter extends KillableSubsystem {

  private SparkMax motor;

  public CoralShooter() {
    motor = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    toggle(CoralShooterStates.OFF); // initialize as off
    ShuffleboardUI.Test.addSlider(
            "Coral Shooter", motor.get(), -1, 1)
        .subscribe(motor::set);
  }

  public enum CoralShooterStates {
    OUT,
    INTAKE,
    OFF;
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    motor.set(speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralShooterStates state) {
    switch (state) {
      case OUT:
        toggle(Constants.CoralShooter.OUT_SPEED);
        break;
      case INTAKE:
        toggle(Constants.CoralShooter.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  @Override
  public void kill() {
    toggle(CoralShooterStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
  }
}
