// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardUI;

public class CoralShooter extends KillableSubsystem {

  private SparkMax motor;

  public CoralShooter() {
    motor = new SparkMax(Constants.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    toggle(CoralShooterStates.OFF); // initialize as off
    ShuffleboardUI.Test.addSlider(
            "Coral Shooter", motor.get(), -1, 1) // LEFT set slider to show value between -1 and 1
        .subscribe(motor::set); // LEFT if the slider is moved, call flywheelL.set
  }

  public enum CoralShooterStates {
    OUT,
    INTAKE,
    OFF;
  }

  /** Set the current shooter speed to speedL and speedR */
  public void toggle(double speedL, double speedR) {
    flywheelL.set(-speedL); // left side is inverted because it is facing the other way
    flywheelR.set(speedR);
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    toggle(speed, speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralShooterStates state) {
    switch (state) {
      case SPEAKER: // High speed for speaker
        toggle(Constants.Shooter.SPEAKER_SPEED);
        break;
      case AMP: // Low speed for amp
        toggle(Constants.Shooter.AMP_SPEED);
        break;
      case REVERSE: // Reverse for note acquisition
        toggle(Constants.Shooter.REVERSE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0); // set speed to 0
        break;
    }
  }

  @Override
  public void kill() {
    toggle(CoralShooterStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    flywheelL.close();
    flywheelR.close();
  }
}
