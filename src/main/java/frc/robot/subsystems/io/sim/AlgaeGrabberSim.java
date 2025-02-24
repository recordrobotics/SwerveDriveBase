package frc.robot.subsystems.io.sim;

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
import frc.robot.subsystems.io.AlgaeGrabberIO;

public class AlgaeGrabberSim implements AlgaeGrabberIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final SparkMaxSim wheelSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.AlgaeGrabber.kV, Constants.AlgaeGrabber.kA),
          wheelMotor,
          0.01,
          0.01);

  private final DigitalInput algaeDetector =
      new DigitalInput(RobotMap.AlgaeGrabber.LIMIT_SWITCH_ID);
  private final SimDevice algaeDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.AlgaeGrabber.LIMIT_SWITCH_ID);
  private final SimBoolean algaeDetectorSimValue;

  public AlgaeGrabberSim(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.AlgaeGrabber.MOTOR_ID, MotorType.kBrushless);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);

    if (algaeDetectorSim != null)
      algaeDetectorSimValue = algaeDetectorSim.createBoolean("Value", Direction.kOutput, true);
    else algaeDetectorSimValue = null;

    if (algaeDetectorSim != null) algaeDetector.setSimDevice(algaeDetectorSim);
    else algaeDetector.close();
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
  public boolean getAlgaeDetector() {
    // TODO: algaeDetector.get() does not update
    if (algaeDetectorSim != null) return algaeDetectorSimValue.get();
    else return false;
  }

  public void setAlgaeDetectorSim(boolean newValue) {
    if (algaeDetectorSimValue != null) algaeDetectorSimValue.set(newValue);
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    if (algaeDetectorSim != null) {
      algaeDetectorSim.close();
      algaeDetector.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    wheelSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()) * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }
}
