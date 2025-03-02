package frc.robot.subsystems.io.real;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.AlgaeGrabberIO;

public class AlgaeGrabberReal implements AlgaeGrabberIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final SparkMax wheel;

  public AlgaeGrabberReal(double periodicDt) {
    this.periodicDt = periodicDt;
    wheel = new SparkMax(RobotMap.AlgaeGrabber.MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void setWheelVoltage(double outputVolts) {
    wheel.setVoltage(outputVolts);
  }

  @Override
  public void setWheelPosition(double newValue) {
    wheel.getEncoder().setPosition(newValue);
  }

  @Override
  public double getWheelPosition() {
    return wheel.getEncoder().getPosition();
  }

  @Override
  public double getWheelVelocity() {
    return wheel.getEncoder().getVelocity();
  }

  @Override
  public double getWheelVoltage() {
    return wheel.getAppliedOutput() * wheel.getBusVoltage();
  }

  @Override
  public void setWheelPercent(double newValue) {
    wheel.set(newValue);
  }

  @Override
  public double getWheelPercent() {
    return wheel.get();
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheel.getOutputCurrent();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
  }

  @Override
  public void simulationPeriodic() {}
}
