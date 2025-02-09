package frc.robot.subsystems.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CoralShooterReal implements CoralShooterIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final SparkMax wheel;
  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);

  public CoralShooterReal(double periodicDt) {
    this.periodicDt = periodicDt;
    wheel = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
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
  public boolean getCoralDetector() {
    return coralDetector.get();
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheel.getOutputCurrent();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    coralDetector.close();
  }

  @Override
  public void simulationPeriodic() {}
}
