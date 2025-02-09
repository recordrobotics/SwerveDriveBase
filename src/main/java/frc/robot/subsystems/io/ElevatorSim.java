package frc.robot.subsystems.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class ElevatorSim implements ElevatorIO {

  private final double periodicDt;

  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final TalonFXSimState motorLeftSim;
  private final TalonFXSimState motorRightSim;

  private final edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim =
      new edu.wpi.first.wpilibj.simulation.ElevatorSim(
          DCMotor.getKrakenX60(2),
          Constants.Elevator.GEAR_RATIO,
          5.137924871015,
          Constants.Elevator.DRUM_RADIUS,
          0,
          Constants.Elevator.MAX_HEIGHT,
          true,
          0,
          0.001,
          0.001);

  private final DigitalInput bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
  private final DigitalInput topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);
  private final SimDevice bottomEndStopSim =
      SimDevice.create("DigitalInput", RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
  private final SimDevice topEndStopSim =
      SimDevice.create("DigitalInput", RobotMap.Elevator.TOP_ENDSTOP_ID);
  private final SimBoolean bottomEndStopSimValue;
  private final SimBoolean topEndStopSimValue;

  public ElevatorSim(double periodicDt) {
    this.periodicDt = periodicDt;

    motorLeft = new TalonFX(RobotMap.Elevator.MOTOR_LEFT_ID);
    motorRight = new TalonFX(RobotMap.Elevator.MOTOR_RIGHT_ID);

    motorLeftSim = motorLeft.getSimState();
    motorRightSim = motorRight.getSimState();

    bottomEndStopSimValue = bottomEndStopSim.createBoolean("Value", Direction.kOutput, false);
    topEndStopSimValue = topEndStopSim.createBoolean("Value", Direction.kOutput, false);

    bottomEndStop.setSimDevice(bottomEndStopSim);
    topEndStop.setSimDevice(topEndStopSim);
  }

  @Override
  public void setLeftMotorVoltage(double outputVolts) {
    motorLeft.setVoltage(outputVolts);
  }

  @Override
  public void setRightMotorVoltage(double outputVolts) {
    motorRight.setVoltage(outputVolts);
  }

  @Override
  public double getLeftMotorVoltage() {
    return motorLeft.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getRightMotorVoltage() {
    return motorRight.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPosition(double newValue) {
    motorLeft.setPosition(newValue);
  }

  @Override
  public void setRightMotorPosition(double newValue) {
    motorRight.setPosition(newValue);
  }

  @Override
  public double getLeftMotorPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  @Override
  public double getLeftMotorVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  @Override
  public double getRightMotorPosition() {
    return motorRight.getPosition().getValueAsDouble();
  }

  @Override
  public double getRightMotorVelocity() {
    return motorRight.getVelocity().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPercent(double newValue) {
    motorLeft.set(newValue);
  }

  @Override
  public void setRightMotorPercent(double newValue) {
    motorRight.set(newValue);
  }

  @Override
  public double getLeftMotorPercent() {
    return motorLeft.get();
  }

  @Override
  public double getRightMotorPercent() {
    return motorRight.get();
  }

  // TODO: weird bug where endstop.get() doesn't update in simulation

  @Override
  public boolean getTopEndStop() {
    return topEndStopSimValue.get();
    // return topEndStop.get();
  }

  @Override
  public boolean getBottomEndStop() {
    return bottomEndStopSimValue.get();
    // return bottomEndStop.get();
  }

  public void setTopEndStopSim(boolean newValue) {
    topEndStopSimValue.set(newValue);
  }

  public void setBottomEndStopSim(boolean newValue) {
    bottomEndStopSimValue.set(newValue);
  }

  @Override
  public double getLeftMotorCurrentDraw() {
    return motorLeft.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getRightMotorCurrentDraw() {
    return motorRight.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motorLeft.close();
    motorRight.close();
    bottomEndStop.close();
    topEndStop.close();
    bottomEndStopSim.close();
    topEndStopSim.close();
  }

  @Override
  public void simulationPeriodic() {
    motorLeftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorRightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorLeftVoltage = motorLeftSim.getMotorVoltage();
    var motorRightVoltage = motorRightSim.getMotorVoltage();

    elevatorSim.setInputVoltage((motorLeftVoltage + motorRightVoltage) / 2.0);
    elevatorSim.update(periodicDt);

    motorLeftSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
    motorLeftSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
    motorRightSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
    motorRightSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
  }
}
