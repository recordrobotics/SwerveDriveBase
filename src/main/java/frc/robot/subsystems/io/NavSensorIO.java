package frc.robot.subsystems.io;

public interface NavSensorIO extends AutoCloseable {

  public void reset();

  public void resetDisplacement();

  public double getAngle();

  public double getWorldLinearAccelX();

  public double getWorldLinearAccelY();

  public boolean isConnected();

  public void simulationPeriodic();
}
