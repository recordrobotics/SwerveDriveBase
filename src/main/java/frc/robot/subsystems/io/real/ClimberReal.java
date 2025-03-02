package frc.robot.subsystems.io.real;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberReal implements ClimberIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final TalonFX motor;

  public ClimberReal(double periodicDt) {
    this.periodicDt = periodicDt;

    motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {
    motor.getConfigurator().apply(configuration);
  }

  @Override
  public void setVoltage(Voltage outputVolts) {
    motor.setVoltage(outputVolts.in(Volts));
  }

  @Override
  public void setPosition(double newValue) {
    motor.setPosition(newValue);
  }

  @Override
  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setPercent(double newValue) {
    motor.set(newValue);
  }

  @Override
  public double getPercent() {
    return motor.get();
  }

  @Override
  public double getCurrentDrawAmps() {
    return motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }

  @Override
  public void simulationPeriodic() {}
}
