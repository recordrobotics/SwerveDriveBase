package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO extends AutoCloseable {
  public void applyTalonFXConfig(TalonFXConfiguration configuration);

  public void setVoltage(Voltage outputVolts);

  public void setPosition(double newValue);

  public double getPosition();

  public double getVelocity();

  public void setPercent(double newValue);

  public double getPercent();

  public double getCurrentDrawAmps();

  public void simulationPeriodic();
}
