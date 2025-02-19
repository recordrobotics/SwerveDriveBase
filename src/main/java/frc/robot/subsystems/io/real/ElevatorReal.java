package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorReal implements ElevatorIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final TalonFX motorLeft;
  private final TalonFX motorRight;
  private final DigitalInput bottomEndStop;
  private final DigitalInput topEndStop;

  public ElevatorReal(double periodicDt) {
    this.periodicDt = periodicDt;

    motorLeft = new TalonFX(RobotMap.Elevator.MOTOR_LEFT_ID);
    motorRight = new TalonFX(RobotMap.Elevator.MOTOR_RIGHT_ID);
    bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
    topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {
    motorLeft.getConfigurator().apply(configuration);
    motorRight.getConfigurator().apply(configuration);
  }

  @Override
  public void setLeftMotorVoltage(double outputVolts) {
    motorLeft.setVoltage(outputVolts);
  }

  @Override
  public void setRightMotorVoltage(double outputVolts) {
    motorRight.setVoltage(outputVolts);
  }

  @Override
  public double getLeftMotorVoltage() {
    return motorLeft.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getRightMotorVoltage() {
    return motorRight.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPosition(double newValue) {
    motorLeft.setPosition(newValue);
  }

  @Override
  public void setRightMotorPosition(double newValue) {
    motorRight.setPosition(newValue);
  }

  @Override
  public double getLeftMotorPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  @Override
  public double getLeftMotorVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  @Override
  public double getRightMotorPosition() {
    return motorRight.getPosition().getValueAsDouble();
  }

  @Override
  public double getRightMotorVelocity() {
    return motorRight.getVelocity().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPercent(double newValue) {
    motorLeft.set(newValue);
  }

  @Override
  public void setRightMotorPercent(double newValue) {
    motorRight.set(newValue);
  }

  @Override
  public double getLeftMotorPercent() {
    return motorLeft.get();
  }

  @Override
  public double getRightMotorPercent() {
    return motorRight.get();
  }

  @Override
  public boolean getTopEndStop() {
    return topEndStop.get();
  }

  @Override
  public boolean getBottomEndStop() {
    return bottomEndStop.get();
  }

  @Override
  public double getLeftMotorCurrentDraw() {
    return motorLeft.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getRightMotorCurrentDraw() {
    return motorRight.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motorLeft.close();
    motorRight.close();
    bottomEndStop.close();
    topEndStop.close();
  }

  @Override
  public void simulationPeriodic() {}
}
