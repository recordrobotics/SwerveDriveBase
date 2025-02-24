// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.AlgaeGrabberIO;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeGrabber extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final AlgaeGrabberIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.AlgaeGrabber.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final PIDController pid =
      new PIDController(
          Constants.AlgaeGrabber.kP, Constants.AlgaeGrabber.kI, Constants.AlgaeGrabber.kD);
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.AlgaeGrabber.kS, Constants.AlgaeGrabber.kV, Constants.AlgaeGrabber.kA);

  public AlgaeGrabber(AlgaeGrabberIO io) {
    this.io = io;

    toggle(AlgaeGrabberStates.OFF); // initialize as off

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("AlgaeGrabber/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setWheelVoltage(v.in(Volts)), null, this));
  }

  private final SysIdRoutine sysIdRoutine;

  public enum AlgaeGrabberStates {
    OUT,
    INTAKE,
    OFF;
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return io.getWheelVelocity() / 60.0 / Constants.AlgaeGrabber.GEAR_RATIO; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return io.getWheelPosition() / Constants.AlgaeGrabber.GEAR_RATIO;
  }

  @AutoLogOutput
  public double getWheelVoltage() {
    return io.getWheelVoltage();
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(AlgaeGrabberStates state) {
    switch (state) {
      case OUT:
        toggle(Constants.AlgaeGrabber.OUT_SPEED);
        break;
      case INTAKE:
        toggle(Constants.AlgaeGrabber.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  @AutoLogOutput
  public boolean hasAlgae() {
    return debounced_value;
  }

  private double lastSpeed = 0;

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control

    lastSpeed = pid.getSetpoint();
    debounced_value = !m_debouncer.calculate(io.getAlgaeDetector());
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Algae on the Elevator", io.getWheelPercent(), -1, 1)
        .subscribe(io::setWheelPercent);
  }

  @Override
  public void kill() {
    toggle(AlgaeGrabberStates.OFF);
    io.setWheelVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getWheelCurrentDrawAmps();
  }
}
