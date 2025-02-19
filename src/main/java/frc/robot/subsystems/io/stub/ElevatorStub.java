package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorStub implements ElevatorIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public ElevatorStub(double periodicDt) {
    this.periodicDt = periodicDt;
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

  @Override
  public void setLeftMotorVoltage(double outputVolts) {}

  @Override
  public void setRightMotorVoltage(double outputVolts) {}

  @Override
  public double getLeftMotorVoltage() {
    return 0;
  }

  @Override
  public double getRightMotorVoltage() {
    return 0;
  }

  @Override
  public void setLeftMotorPosition(double newValue) {}

  @Override
  public void setRightMotorPosition(double newValue) {}

  @Override
  public double getLeftMotorPosition() {
    return 0;
  }

  @Override
  public double getLeftMotorVelocity() {
    return 0;
  }

  @Override
  public double getRightMotorPosition() {
    return 0;
  }

  @Override
  public double getRightMotorVelocity() {
    return 0;
  }

  @Override
  public void setLeftMotorPercent(double newValue) {}

  @Override
  public void setRightMotorPercent(double newValue) {}

  @Override
  public double getLeftMotorPercent() {
    return 0;
  }

  @Override
  public double getRightMotorPercent() {
    return 0;
  }

  @Override
  public boolean getTopEndStop() {
    return false;
  }

  @Override
  public boolean getBottomEndStop() {
    return false;
  }

  @Override
  public double getLeftMotorCurrentDraw() {
    return 0;
  }

  @Override
  public double getRightMotorCurrentDraw() {
    return 0;
  }

  @Override
  public void close() throws Exception {}

  @Override
  public void simulationPeriodic() {}
}
