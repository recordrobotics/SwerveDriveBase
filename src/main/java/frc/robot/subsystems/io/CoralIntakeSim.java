package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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

public class CoralIntakeSim implements CoralIntakeIO {

  private final double periodicDt;

  private final SparkMax wheel;
  private final TalonFX arm;

  private final SparkMaxSim wheelSim;
  private final TalonFXSimState armSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);
  private final DCMotor armMotor = DCMotor.getFalcon500(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralIntake.kV, Constants.CoralIntake.kA),
          wheelMotor,
          0.01,
          0.01);

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralIntake.sV, Constants.CoralIntake.sA),
          armMotor,
          Constants.CoralIntake.ARM_GEAR_RATIO,
          Units.inchesToMeters(19),
          -1.1,
          Math.PI / 2,
          true,
          Constants.CoralIntake.ARM_START_POS,
          0.001,
          0.001);

  private final DigitalInput coralDetector = new DigitalInput(RobotMap.CoralIntake.LIMIT_SWITCH_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.CoralIntake.LIMIT_SWITCH_ID);
  private final SimBoolean coralDetectorSimValue;

  public CoralIntakeSim(double periodicDt) {
    this.periodicDt = periodicDt;

    wheel = new SparkMax(RobotMap.CoralIntake.WHEEL_ID, MotorType.kBrushless);
    arm = new TalonFX(RobotMap.CoralIntake.ARM_ID);
    wheelSim = new SparkMaxSim(wheel, wheelMotor);
    armSim = arm.getSimState();

    if (coralDetectorSim != null)
      coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
    else coralDetectorSimValue = null;

    if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
    else coralDetector.close();
  }

  @Override
  public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
    arm.getConfigurator().apply(configuration);
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
    arm.setPosition(newValue);
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
    return arm.getPosition().getValueAsDouble();
  }

  @Override
  public double getArmVelocity() {
    return arm.getVelocity().getValueAsDouble();
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
  public boolean getCoralDetector() {
    // TODO: coralDetector.get() does not update
    if (coralDetectorSim != null) return coralDetectorSimValue.get();
    else return false;
  }

  public void setCoralDetectorSim(boolean newValue) {
    if (coralDetectorSimValue != null) coralDetectorSimValue.set(newValue);
  }

  @Override
  public double getWheelCurrentDrawAmps() {
    return wheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public double getArmCurrentDrawAmps() {
    return arm.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    wheel.close();
    arm.close();
    if (coralDetectorSim != null) {
      coralDetectorSim.close();
      coralDetector.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    armSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var wheelVoltage = wheelSim.getAppliedOutput() * wheelSim.getBusVoltage();
    var armVoltage = armSim.getMotorVoltage();

    wheelSimModel.setInputVoltage(wheelVoltage);
    wheelSimModel.update(periodicDt);

    armSimModel.setInputVoltage(armVoltage);
    armSimModel.update(periodicDt);

    wheelSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec()) * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);

    armSim.setRawRotorPosition(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(
                armSimModel.getAngleRads() - Constants.CoralIntake.ARM_START_POS));
    armSim.setRotorVelocity(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
  }
}
