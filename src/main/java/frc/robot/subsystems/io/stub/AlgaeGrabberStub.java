package frc.robot.subsystems.io.stub;

import frc.robot.subsystems.io.AlgaeGrabberIO;

public class AlgaeGrabberStub implements AlgaeGrabberIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public AlgaeGrabberStub(double periodicDt) {
    this.periodicDt = periodicDt;
  }

  @Override
  public void setWheelVoltage(double outputVolts) {}

  @Override
  public void setWheelPosition(double newValue) {}

  @Override
  public double getWheelPosition() {
    return 0;
  }

  @Override
  public double getWheelVelocity() {
    return 0;
  }

  @Override
  public double getWheelVoltage() {
    return 0;
  }

  @Override
  public void setWheelPercent(double newValue) {}

  @Override
  public double getWheelPercent() {
    return 0;
  }

  @Override
  public boolean getAlgaeDetector() {
    return false;
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return 0;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void simulationPeriodic() {}
}
