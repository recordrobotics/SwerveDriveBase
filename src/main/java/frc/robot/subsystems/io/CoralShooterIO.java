package frc.robot.subsystems.io;

public interface CoralShooterIO extends AutoCloseable {

  public void setTopWheelVoltage(double outputVolts);
  public void setBottomWheelVoltage(double outputVolts);

  public void setTopWheelPosition(double newValue);
  public void setBottomWheelPosition(double newValue);

  public double getTopWheelPosition();
  public double getBottomWheelPosition();

  public double getTopWheelVelocity();
  public double getBottomWheelVelocity();

  public double getTopWheelVoltage();
  public double getBottomWheelVoltage();

  public void setTopWheelPercent(double newValue);
  public void setBottomWheelPercent(double newValue);

  public double getTopWheelPercent();
  public double getBottomWheelPercent();

  public boolean getCoralDetector();

  public double getWheelsCurrentDrawAmps();

  public void simulationPeriodic();
}
