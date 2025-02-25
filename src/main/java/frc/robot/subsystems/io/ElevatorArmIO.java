package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface ElevatorArmIO extends AutoCloseable {

  public void applyArmTalonFXConfig(TalonFXConfiguration configuration);

  public void setArmVoltage(double outputVolts);

  public void setArmPosition(double newValue);

  public double getArmPosition();

  public double getArmVelocity();

  public void setArmPercent(double newValue);

  public double getArmPercent();

  public double getArmVoltage();

  public double getArmCurrentDrawAmps();

  public void simulationPeriodic();
}
