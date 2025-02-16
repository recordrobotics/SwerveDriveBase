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

  private final SparkMax spark;
  private final SparkMaxSim sparkSim;

  private final DCMotor motor = DCMotor.getNeo550(1);

  private final DCMotorSim SimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Constants.CoralShooter.kV, Constants.CoralShooter.kA),
          motor,
          0.01,
          0.01);

  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private final SimBoolean coralDetectorSimValue;

  public CoralShooterSim(double periodicDt) {
    this.periodicDt = periodicDt;

    spark = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    sparkSim = new SparkMaxSim(spark, motor);

    if (coralDetectorSim != null)
      coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
    else coralDetectorSimValue = null;

    if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
    else coralDetector.close();
  }

  @Override
  public void setVoltage(double outputVolts) {
    spark.setVoltage(outputVolts);
  }

  @Override
  public void setPosition(double newValue) {
    spark.getEncoder().setPosition(newValue);
  }

  @Override
  public double getPosition() {
    return spark.getEncoder().getPosition();
  }

  @Override
  public double getVelocity() {
    return spark.getEncoder().getVelocity();
  }

  @Override
  public double getVoltage() {
    return spark.getAppliedOutput();
  }

  @Override
  public void setPercent(double newValue) {
    spark.set(newValue);
  }

  @Override
  public double getPercent() {
    return spark.get();
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
  public double getCurrentDrawAmps() {
    return SimModel.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {
    spark.close();
    if (coralDetectorSim != null) {
      coralDetectorSim.close();
      coralDetector.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    var Voltage = sparkSim.getAppliedOutput() * sparkSim.getBusVoltage();

    SimModel.setInputVoltage(Voltage);
    SimModel.update(periodicDt);

    sparkSim.iterate(
        Units.radiansToRotations(SimModel.getAngularVelocityRadPerSec()) * 60.0,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }
}
