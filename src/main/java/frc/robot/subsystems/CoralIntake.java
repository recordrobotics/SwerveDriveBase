// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.CoralIntakeIO;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends KillableSubsystem implements ShuffleboardPublisher {

  private final CoralIntakeIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final ProfiledPIDController armPID =
      new ProfiledPIDController(
          Constants.CoralIntake.sP,
          Constants.CoralIntake.sI,
          Constants.CoralIntake.sD,
          new Constraints(
              Constants.CoralIntake.MAX_ARM_VELOCITY, Constants.CoralIntake.MAX_ARM_ACCELERATION));
  private final ArmFeedforward armFeedForward =
      new ArmFeedforward(
          Constants.CoralIntake.sS,
          Constants.CoralIntake.sG,
          Constants.CoralIntake.sV,
          Constants.CoralIntake.sA);

  private final PIDController pid =
      new PIDController(
          Constants.CoralIntake.kP, Constants.CoralIntake.kI, Constants.CoralIntake.kD);

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.CoralIntake.kS, Constants.CoralIntake.kV, Constants.CoralIntake.kA);

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;

    io.setArmPosition(0);
    toggle(CoralIntakeStates.OFF);

    sysIdRoutineWheel =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default 1 volt/second ramp rate
                null, // default 7 volt step voltage
                null,
                (state ->
                    Logger.recordOutput("CoralIntake/Wheel/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setWheelVoltage(v.magnitude()), null, this));

    sysIdRoutineArm =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(3).per(Second),
                Volts.of(1.7),
                Seconds.of(1),
                (state -> Logger.recordOutput("CoralIntake/Arm/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setArmVoltage(v.magnitude()), null, this));

    SmartDashboard.putNumber("CoralIntakeArm", 0);
  }

  private final SysIdRoutine sysIdRoutineWheel;
  private final SysIdRoutine sysIdRoutineArm;

  public enum CoralIntakeStates {
    REVERSE,
    INTAKE,
    OFF;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return debounced_value;
  }

  public enum IntakeArmStates {
    UP,
    DOWN,
    OFF;
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return io.getWheelVelocity() / 60.0; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return io.getWheelPosition();
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getArmPosition() / Constants.CoralIntake.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getArmVelocity() / Constants.CoralIntake.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getArmPercent() * RobotController.getBatteryVoltage();
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  public void toggleArm(double angleRadians) {
    armPID.setGoal(angleRadians);
  }

  public boolean armAtGoal() {
    return armPID.atGoal();
  }

  public void toggleArm(IntakeArmStates state) {
    switch (state) {
      case UP:
        toggleArm(Constants.CoralIntake.ARM_UP);
        break;
      case DOWN:
        toggleArm(Constants.CoralIntake.ARM_DOWN);
        break;
      case OFF:
      default:
        io.setArmVoltage(0);
        break;
    }
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralIntakeStates state) {
    switch (state) {
      case REVERSE:
        toggle(Constants.CoralIntake.REVERSE_SPEED);
        break;
      case INTAKE:
        toggle(Constants.CoralIntake.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private double lastSpeed = 0; // TODO why is this here?

  @Override
  public void periodic() {
    toggleArm(SmartDashboard.getNumber("CoralIntakeArm", 0));

    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    lastSpeed = pid.getSetpoint();

    double pidOutputArm = armPID.calculate(getArmAngle());

    double armFeedforwardOutput =
        armFeedForward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, armPID.getSetpoint().velocity);

    Logger.recordOutput("CoralArmTargetPosition", armPID.getSetpoint().position);
    Logger.recordOutput("CoralArmTargetVelocity", armPID.getSetpoint().velocity);
    Logger.recordOutput("CoralIntakeSetVoltage", pidOutputArm);
    Logger.recordOutput("CoralIntakeSetVoltageFF", armFeedforwardOutput);

    io.setArmVoltage(pidOutputArm + armFeedforwardOutput);
    currentSetpoint = armPID.getSetpoint();

    // Update mechanism
    RobotContainer.model.coralIntake.update(getArmAngle());
    RobotContainer.model.coralIntake.updateSetpoint(currentSetpoint.position);

    debounced_value = !m_debouncer.calculate(io.getCoralDetector());
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command sysIdQuasistaticWheel(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.quasistatic(direction);
  }

  public Command sysIdDynamicWheel(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.dynamic(direction);
  }

  public Command sysIdQuasistaticArm(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.quasistatic(direction);
  }

  public Command sysIdDynamicArm(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Coral Intake Motor", io.getWheelPercent(), -1, 1)
        .subscribe(io::setWheelPercent);
    DashboardUI.Test.addSlider("Coral Intake Arm Pos", io.getArmPosition(), -1, 1)
        .subscribe(this::toggleArm);
  }

  @Override
  public void kill() {
    toggle(CoralIntakeStates.OFF);
    io.setWheelVoltage(0);
    io.setArmVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }
}
