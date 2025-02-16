package frc.robot.subsystems.io;

public interface CoralShooterIO extends AutoCloseable {

  public void setVoltage(double outputVolts);

  public void setPosition(double newValue);

  public double getPosition();

  public double getVelocity();

  public double getVoltage();

  public void setPercent(double newValue);

  public double getPercent();

  public boolean getCoralDetector();

  public double getCurrentDrawAmps();

  public void simulationPeriodic();
}
