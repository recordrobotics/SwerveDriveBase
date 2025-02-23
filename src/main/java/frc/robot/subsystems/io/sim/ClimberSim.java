package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberSim implements ClimberIO {

  private final double periodicDt;

  private final TalonFX motor;

  private final TalonFXSimState motorSim;

  private final DCMotor theMotor = DCMotor.getKrakenX60(1);

  private final SingleJointedArmSim simModel =
      new SingleJointedArmSim(
          LinearSystemId.createDCMotorSystem(Constants.Climber.kV, Constants.Climber.kA),
          theMotor,
          Constants.Climber.GEAR_RATIO,
          Units.inchesToMeters(88), // TODO: make sure gear ratio is correct
          Units.degreesToRadians(-95), // TODO: is bad numbers
          Units.degreesToRadians(95),
          true,
          Constants.Climber.START_POS,
          0.001,
          0.001);

  public ClimberSim(double periodicDt) {
    this.periodicDt = periodicDt;

    motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
    motorSim = motor.getSimState();
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {
    motor.getConfigurator().apply(configuration);
  }

  @Override
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
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
  public void simulationPeriodic() {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double voltage = motorSim.getMotorVoltage();

    simModel.setInputVoltage(voltage);
    simModel.update(periodicDt);

    motorSim.setRawRotorPosition(
        Constants.Climber.GEAR_RATIO
            * Units.radiansToRotations(simModel.getAngleRads() - Constants.Climber.START_POS));
    motorSim.setRotorVelocity(
        Constants.Climber.GEAR_RATIO * Units.radiansToRotations(simModel.getVelocityRadPerSec()));
  }
}
