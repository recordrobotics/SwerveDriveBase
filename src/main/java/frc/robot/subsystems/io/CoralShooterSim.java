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

  private final SparkMax topWheel;
  private final SparkMax bottomWheel;
  private final SparkMaxSim topWheelSim;
  private final SparkMaxSim bottomWheelSim;

  private final DCMotor topWheelMotor = DCMotor.getNeo550(1);
  private final DCMotor bottomWheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim topWheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralShooter.kV, Constants.CoralShooter.kA),
          topWheelMotor,
          0.01,
          0.01);
  private final DCMotorSim bottomWheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralShooter.kV, Constants.CoralShooter.kA),
          bottomWheelMotor,
          0.01,
          0.01);

  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimBoolean coralDetectorSimValue;

  public CoralShooterSim(double periodicDt) {
    this.periodicDt = periodicDt;

    topWheel = new SparkMax(RobotMap.CoralShooter.TOP_MOTOR_ID, MotorType.kBrushless);
    bottomWheel = new SparkMax(RobotMap.CoralShooter.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    topWheelSim = new SparkMaxSim(topWheel, topWheelMotor);
    bottomWheelSim = new SparkMaxSim(bottomWheel, bottomWheelMotor);

    if (coralDetectorSim != null)
      coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
    else coralDetectorSimValue = null;

    if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
    else coralDetector.close();
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
    // TODO: coralDetector.get() does not update
    if (coralDetectorSim != null) return coralDetectorSimValue.get();
    else return false;
  }

  public void setCoralDetectorSim(boolean newValue) {
    if (coralDetectorSimValue != null) coralDetectorSimValue.set(newValue);
  }

  @Override
  public double getWheelsCurrentDrawAmps() {
    return topWheelSimModel.getCurrentDrawAmps() + bottomWheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {
    topWheel.close();
    bottomWheel.close();
    if (coralDetectorSim != null) {
      coralDetectorSim.close();
      coralDetector.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    var topWheelVoltage = topWheelSim.getAppliedOutput() * topWheelSim.getBusVoltage();
    var bottomWheelVoltage = bottomWheelSim.getAppliedOutput() * bottomWheelSim.getBusVoltage();

    topWheelSimModel.setInputVoltage(topWheelVoltage);
    topWheelSimModel.update(periodicDt);
    bottomWheelSimModel.setInputVoltage(bottomWheelVoltage);
    bottomWheelSimModel.update(periodicDt);

    topWheelSim.iterate(
        Units.radiansToRotations(topWheelSimModel.getAngularVelocityRadPerSec()) * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);
    bottomWheelSim.iterate(
        Units.radiansToRotations(bottomWheelSimModel.getAngularVelocityRadPerSec()) * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }
}
