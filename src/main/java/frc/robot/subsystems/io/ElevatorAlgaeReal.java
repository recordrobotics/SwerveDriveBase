package frc.robot.subsystems.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class ElevatorAlgaeReal implements ElevatorAlgaeIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final SparkMax wheel;
  private final DigitalInput algaeDetector =
      new DigitalInput(RobotMap.ElevatorAlgae.LIMIT_SWITCH_ID);

  public ElevatorAlgaeReal(double periodicDt) {
    this.periodicDt = periodicDt;
    wheel = new SparkMax(RobotMap.ElevatorAlgae.MOTOR_ID, MotorType.kBrushless);
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
    return wheel.getAppliedOutput();
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
  public boolean getAlgaeDetector() {
    return algaeDetector.get();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    algaeDetector.close();
  }

  @Override
  public void simulationPeriodic() {}
}
