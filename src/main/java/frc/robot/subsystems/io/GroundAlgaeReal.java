package frc.robot.subsystems.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class GroundAlgaeReal implements GroundAlgaeIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final SparkMax wheel;
  private final SparkMax arm;
  private final DigitalInput algaeDetector = new DigitalInput(RobotMap.GroundAlgae.LIMIT_SWITCH_ID);

  public GroundAlgaeReal(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.GroundAlgae.MOTOR_ID, MotorType.kBrushless);
    arm = new SparkMax(RobotMap.GroundAlgae.ARM_ID, MotorType.kBrushless);
  }

  @Override
  public void setWheelVoltage(double outputVolts) {
    wheel.setVoltage(outputVolts);
  }

  @Override
  public void setArmVoltage(double outputVolts) {
    arm.setVoltage(outputVolts);
  }

  @Override
  public void setWheelPosition(double newValue) {
    wheel.getEncoder().setPosition(newValue);
  }

  @Override
  public void setArmPosition(double newValue) {
    arm.getEncoder().setPosition(newValue);
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
  public double getArmPosition() {
    return arm.getEncoder().getPosition();
  }

  @Override
  public double getArmVelocity() {
    return arm.getEncoder().getVelocity();
  }

  @Override
  public void setWheelPercent(double newValue) {
    wheel.set(newValue);
  }

  @Override
  public void setArmPercent(double newValue) {
    arm.set(newValue);
  }

  @Override
  public double getWheelPercent() {
    return wheel.get();
  }

  @Override
  public double getArmPercent() {
    return arm.get();
  }

  @Override
  public boolean getAlgaeDetector() {
    return algaeDetector.get();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    arm.close();
    algaeDetector.close();
  }

  @Override
  public void simulationPeriodic() {}
}
