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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class GroundAlgaeSim implements GroundAlgaeIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final SparkMax arm;

  private final SparkMaxSim wheelSim;
  private final SparkMaxSim armSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);
  private final DCMotor armMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.GroundAlgae.kV, Constants.GroundAlgae.kA),
          wheelMotor,
          0.01,
          0.01);

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          LinearSystemId.createDCMotorSystem(Constants.GroundAlgae.sV, Constants.GroundAlgae.sA),
          armMotor,
          Constants.GroundAlgae.ARM_GEAR_RATIO,
          Units.inchesToMeters(28),
          -1.1,
          Math.PI,
          true,
          0,
          0.001,
          0.001);

  private final DigitalInput algaeDetector = new DigitalInput(RobotMap.GroundAlgae.LIMIT_SWITCH_ID);
  private final SimDevice algaeDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.GroundAlgae.LIMIT_SWITCH_ID);
  private final SimBoolean algaeDetectorSimValue;

  public GroundAlgaeSim(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.GroundAlgae.MOTOR_ID, MotorType.kBrushless);
    arm = new SparkMax(RobotMap.GroundAlgae.ARM_ID, MotorType.kBrushless);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);
    armSim = new SparkMaxSim(arm, armMotor);

    algaeDetectorSimValue = algaeDetectorSim.createBoolean("Value", Direction.kOutput, false);
    algaeDetector.setSimDevice(algaeDetectorSim);
  }

  @Override
  public void setWheelVoltage(double outputVolts) {
    wheel.setVoltage(outputVolts);
  }

  @Override
  public void setArmVoltage(double outputVolts) {
    arm.setVoltage(outputVolts);
  }

  @Override
  public void setWheelPosition(double newValue) {
    wheel.getEncoder().setPosition(newValue);
  }

  @Override
  public void setArmPosition(double newValue) {
    arm.getEncoder().setPosition(newValue);
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
  public double getArmPosition() {
    return arm.getEncoder().getPosition();
  }

  @Override
  public double getArmVelocity() {
    return arm.getEncoder().getVelocity();
  }

  @Override
  public void setWheelPercent(double newValue) {
    wheel.set(newValue);
  }

  @Override
  public void setArmPercent(double newValue) {
    arm.set(newValue);
  }

  @Override
  public double getWheelPercent() {
    return wheel.get();
  }

  @Override
  public double getArmPercent() {
    return arm.get();
  }

  @Override
  public boolean getAlgaeDetector() {
    return algaeDetector.get();
  }

  public void setAlgaeDetectorSim(boolean newValue) {
    algaeDetectorSimValue.set(newValue);
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public double getArmCurrentDrawAmps() {
    return armSimModel.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    arm.close();
    algaeDetector.close();
    algaeDetectorSim.close();
  }

  @Override
  public void simulationPeriodic() {
    wheelSim.setBusVoltage(RobotController.getBatteryVoltage());
    armSim.setBusVoltage(RobotController.getBatteryVoltage());

    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();
    var armVoltage = armSim.getAppliedOutput() * armSim.getBusVoltage();

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    armSimModel.setInputVoltage(armVoltage);
    armSimModel.update(periodicDt);

    wheelSim.setPosition(wheelSimModel.getAngularPositionRotations());
    wheelSim.setVelocity(Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()));

    armSim.setPosition(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(armSimModel.getAngleRads()));
    armSim.setVelocity(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
  }
}
