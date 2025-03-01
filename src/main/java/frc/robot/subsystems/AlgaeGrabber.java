// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.AlgaeGrabberIO;
import frc.robot.subsystems.io.sim.AlgaeGrabberSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeGrabber extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final AlgaeGrabberIO io;

  private final PIDController pid =
      new PIDController(
          Constants.AlgaeGrabber.kP, Constants.AlgaeGrabber.kI, Constants.AlgaeGrabber.kD);
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.AlgaeGrabber.kS, Constants.AlgaeGrabber.kV, Constants.AlgaeGrabber.kA);

  private boolean hasAlgae = false;
  private boolean waitingForAlgae = false;
  private boolean waitingForIntakeSpeed = false;

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

  public AlgaeGrabberSim getSimIO() throws Exception {
    if (io instanceof AlgaeGrabberSim) {
      return (AlgaeGrabberSim) io;
    } else {
      throw new Exception("AlgaeGrabberIO is not a simulation");
    }
  }

  private final SysIdRoutine sysIdRoutine;

  public enum AlgaeGrabberStates {
    OUT_GROUND,
    OUT_REEF,
    INTAKE_GROUND,
    INTAKE_REEF,
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
      case OUT_GROUND:
        toggle(Constants.AlgaeGrabber.OUT_GROUND_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        break;
      case OUT_REEF:
        toggle(Constants.AlgaeGrabber.OUT_REEF_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        break;
      case INTAKE_GROUND:
        toggle(Constants.AlgaeGrabber.INTAKE_GROUND_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = true;
        hasAlgae = false;
        break;
      case INTAKE_REEF:
        toggle(Constants.AlgaeGrabber.INTAKE_REEF_SPEED);
        waitingForAlgae = false;
        waitingForIntakeSpeed = true;
        hasAlgae = false;
        break;
      case OFF: // Off
      default: // should never happen
        waitingForIntakeSpeed = false;
        waitingForAlgae = false;
        toggle(0);
        break;
    }
  }

  @AutoLogOutput
  public boolean hasAlgae() {
    return hasAlgae;
  }

  private double lastSpeed = 0;

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control

    lastSpeed = pid.getSetpoint();

    if (waitingForIntakeSpeed && Math.abs(getWheelVelocity()) > 0.4) {
      waitingForIntakeSpeed = false;
      waitingForAlgae = true;
    }

    if (waitingForAlgae && Math.abs(getWheelVelocity()) < 0.1) {
      hasAlgae = true;
      waitingForAlgae = false;
    }
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
