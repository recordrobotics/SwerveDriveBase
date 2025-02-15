package frc.robot.subsystems.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CoralShooterReal implements CoralShooterIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final SparkMax topWheel;
  private final SparkMax bottomWheel;
  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);

  public CoralShooterReal(double periodicDt) {
    this.periodicDt = periodicDt;
    topWheel = new SparkMax(RobotMap.CoralShooter.TOP_MOTOR_ID, MotorType.kBrushless);
    bottomWheel = new SparkMax(RobotMap.CoralShooter.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void setTopWheelVoltage(double outputVolts) {
    topWheel.setVoltage(outputVolts);
  }

  @Override
  public void setBottomWheelVoltage(double outputVolts) {
    bottomWheel.setVoltage(outputVolts);
  }

  @Override
  public void setTopWheelPosition(double newValue) {
    topWheel.getEncoder().setPosition(newValue);
  }

  @Override
  public void setBottomWheelPosition(double newValue) {
    bottomWheel.getEncoder().setPosition(newValue);
  }

  @Override
  public double getTopWheelPosition() {
    return topWheel.getEncoder().getPosition();
  }

  @Override
  public double getBottomWheelPosition() {
    return bottomWheel.getEncoder().getPosition();
  }

  @Override
  public double getTopWheelVelocity() {
    return topWheel.getEncoder().getVelocity();
  }

  @Override
  public double getBottomWheelVelocity() {
    return bottomWheel.getEncoder().getVelocity();
  }

  @Override
  public double getTopWheelVoltage() {
    return topWheel.getAppliedOutput();
  }

  @Override
  public double getBottomWheelVoltage() {
    return bottomWheel.getAppliedOutput();
  }

  @Override
  public void setTopWheelPercent(double newValue) {
    topWheel.set(newValue);
  }

  @Override
  public void setBottomWheelPercent(double newValue) {
    bottomWheel.set(newValue);
  }

  @Override
  public double getTopWheelPercent() {
    return topWheel.get();
  }

  @Override
  public double getBottomWheelPercent() {
    return bottomWheel.get();
  }

  @Override
  public boolean getCoralDetector() {
    return coralDetector.get();
  }

  @Override
  public double getWheelsCurrentDrawAmps() {
    return topWheel.getOutputCurrent() + bottomWheel.getOutputCurrent();
  }

  @Override
  public void close() throws Exception {
    topWheel.close();
    bottomWheel.close();
    coralDetector.close();
  }

  @Override
  public void simulationPeriodic() {}
}
