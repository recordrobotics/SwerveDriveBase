// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.CoralShooterIO;
import frc.robot.subsystems.io.CoralShooterSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralShooter extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final CoralShooterIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final PIDController topPid =
      new PIDController(
          Constants.CoralShooter.kP, Constants.CoralShooter.kI, Constants.CoralShooter.kD);
  private final SimpleMotorFeedforward topFeedForward =
      new SimpleMotorFeedforward(
          Constants.CoralShooter.kS, Constants.CoralShooter.kV, Constants.CoralShooter.kA);

  private final PIDController bottomPid =
      new PIDController(
          Constants.CoralShooter.kP, Constants.CoralShooter.kI, Constants.CoralShooter.kD);
  private final SimpleMotorFeedforward bottomFeedForward =
      new SimpleMotorFeedforward(
          Constants.CoralShooter.kS, Constants.CoralShooter.kV, Constants.CoralShooter.kA);

  private CoralShooterStates currentState = CoralShooterStates.OFF;

  public CoralShooter(CoralShooterIO io) {
    this.io = io;

    toggle(CoralShooterStates.OFF); // initialize as off
    DashboardUI.Test.addSlider("Coral Shooter Top", io.getTopWheelPercent(), -1, 1)
        .subscribe(io::setTopWheelPercent);
    DashboardUI.Test.addSlider("Coral Shooter Bottom", io.getBottomWheelPercent(), -1, 1)
        .subscribe(io::setBottomWheelPercent);

    sysIdRoutineTop =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state ->
                    Logger.recordOutput("CoralShooter/Top/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setTopWheelVoltage(v.magnitude()), null, this));
    sysIdRoutineBottom =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state ->
                    Logger.recordOutput("CoralShooter/Bottom/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setBottomWheelVoltage(v.magnitude()), null, this));
  }

  public CoralShooterSim getSimIO() throws Exception {
    if (io instanceof CoralShooterSim) {
      return (CoralShooterSim) io;
    } else {
      throw new Exception("CoralShooterIO is not a simulation");
    }
  }

  private final SysIdRoutine sysIdRoutineTop;
  private final SysIdRoutine sysIdRoutineBottom;

  public enum CoralShooterStates {
    OUT,
    INTAKE,
    OFF;
  }

  @AutoLogOutput
  public double getTopWheelVelocity() {
    return io.getTopWheelVelocity() / 60.0; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getBottomWheelVelocity() {
    return io.getBottomWheelVelocity() / 60.0; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getTopWheelPosition() {
    return io.getTopWheelPosition();
  }

  @AutoLogOutput
  public double getBottomWheelPosition() {
    return io.getBottomWheelPosition();
  }

  @AutoLogOutput
  public double getTopWheelVoltage() {
    return io.getTopWheelVoltage();
  }

  @AutoLogOutput
  public double getBottomWheelVoltage() {
    return io.getBottomWheelVoltage();
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double topSpeed, double bottomSpeed) {
    topPid.setSetpoint(topSpeed);
    bottomPid.setSetpoint(bottomSpeed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralShooterStates state) {
    switch (state) {
      case OUT:
        currentState = CoralShooterStates.OUT;
        toggle(Constants.CoralShooter.OUT_SPEED_TOP, Constants.CoralShooter.OUT_SPEED_BOTTOM);
        break;
      case INTAKE:
        currentState = CoralShooterStates.INTAKE;
        toggle(Constants.CoralShooter.INTAKE_SPEED_TOP, Constants.CoralShooter.INTAKE_SPEED_BOTTOM);
        break;
      case OFF: // Off
      default: // should never happen
        currentState = CoralShooterStates.OFF;
        toggle(0, 0);
        break;
    }
  }

  public CoralShooterStates getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return debounced_value;
  }

  private double lastTopSpeed = 0;
  private double lastBottomSpeed = 0;

  @Override
  public void periodic() {
    double topPidOutput = topPid.calculate(getTopWheelVelocity());
    double topFeedforwardOutput =
        topFeedForward.calculateWithVelocities(lastTopSpeed, topPid.getSetpoint());
    io.setTopWheelVoltage(
        topPidOutput + topFeedforwardOutput); // Feed forward runs on voltage control

    double bottomPidOutput = bottomPid.calculate(getBottomWheelVelocity());
    double bottomFeedforwardOutput =
        bottomFeedForward.calculateWithVelocities(lastBottomSpeed, bottomPid.getSetpoint());
    io.setBottomWheelVoltage(
        bottomPidOutput + bottomFeedforwardOutput); // Feed forward runs on voltage control

    lastTopSpeed = topPid.getSetpoint();
    lastBottomSpeed = bottomPid.getSetpoint();
    debounced_value = !m_debouncer.calculate(io.getCoralDetector());
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command sysIdQuasistaticTop(SysIdRoutine.Direction direction) {
    return sysIdRoutineTop.quasistatic(direction);
  }

  public Command sysIdQuasistaticBottom(SysIdRoutine.Direction direction) {
    return sysIdRoutineBottom.quasistatic(direction);
  }

  public Command sysIdDynamicTop(SysIdRoutine.Direction direction) {
    return sysIdRoutineTop.dynamic(direction);
  }

  public Command sysIdDynamicBottom(SysIdRoutine.Direction direction) {
    return sysIdRoutineBottom.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Coral Shooter Top", io.getTopWheelPercent(), -1, 1)
        .subscribe(io::setTopWheelPercent);
    DashboardUI.Test.addSlider("Coral Shooter Bottom", io.getBottomWheelPercent(), -1, 1)
        .subscribe(io::setBottomWheelPercent);
  }

  @Override
  public void kill() {
    toggle(CoralShooterStates.OFF);
    io.setTopWheelVoltage(0);
    io.setBottomWheelVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getWheelsCurrentDrawAmps();
  }
}
