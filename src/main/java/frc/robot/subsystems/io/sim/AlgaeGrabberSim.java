package frc.robot.subsystems.io.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.AlgaeGrabberIO;

public class AlgaeGrabberSim implements AlgaeGrabberIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final SparkMaxSim wheelSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(wheelMotor, 0.001, Constants.AlgaeGrabber.GEAR_RATIO),
          wheelMotor);

  private boolean hasAlgae = false;

  public AlgaeGrabberSim(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.AlgaeGrabber.MOTOR_ID, MotorType.kBrushless);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);
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
    return wheelSimModel.getCurrentDrawAmps();
  }

  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  @Override
  public void close() throws Exception {
    wheel.close();
  }

  @Override
  public void simulationPeriodic() {
    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();

    if (hasAlgae) {
      wheelVoltage /= 20.0;
    }

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    wheelSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
            * 60.0
            * Constants.AlgaeGrabber.GEAR_RATIO,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }
}
