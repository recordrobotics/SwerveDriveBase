package frc.robot.subsystems.io;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class CoralShooterSim implements CoralShooterIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final SparkMaxSim wheelSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralShooter.kV, Constants.CoralShooter.kA),
          wheelMotor,
          0.01,
          0.01);

  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimBoolean coralDetectorSimValue;

  public CoralShooterSim(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);

    coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, false);
    coralDetector.setSimDevice(coralDetectorSim);
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

  public void setCoralDetectorSim(boolean newValue) {
    coralDetectorSimValue.set(newValue);
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    coralDetector.close();
    coralDetectorSim.close();
  }

  @Override
  public void simulationPeriodic() {
    wheelSim.setBusVoltage(RobotController.getBatteryVoltage());

    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    wheelSim.setPosition(wheelSimModel.getAngularPositionRotations());
    wheelSim.setVelocity(Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));
  }
}
