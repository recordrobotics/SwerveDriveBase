package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface ElevatorIO extends AutoCloseable {

  public void applyTalonFXConfig(TalonFXConfiguration configuration);

  public void setLeftMotorVoltage(double outputVolts);

  public void setRightMotorVoltage(double outputVolts);

  public double getLeftMotorVoltage();

  public double getRightMotorVoltage();

  public void setLeftMotorPosition(double newValue);

  public void setRightMotorPosition(double newValue);

  public double getLeftMotorPosition();

  public double getLeftMotorVelocity();

  public double getRightMotorPosition();

  public double getRightMotorVelocity();

  public void setLeftMotorPercent(double newValue);

  public void setRightMotorPercent(double newValue);

  public double getLeftMotorPercent();

  public double getRightMotorPercent();

  public boolean getTopEndStop();

  public boolean getBottomEndStop();

  public double getLeftMotorCurrentDraw();

  public double getRightMotorCurrentDraw();

  public void simulationPeriodic();
}
