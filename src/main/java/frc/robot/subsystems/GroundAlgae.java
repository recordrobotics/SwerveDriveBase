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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.GroundAlgaeIO;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class GroundAlgae extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final GroundAlgaeIO io;

  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.GroundAlgae.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final ProfiledPIDController armPID =
      new ProfiledPIDController(
          Constants.GroundAlgae.sP,
          Constants.GroundAlgae.sI,
          Constants.GroundAlgae.sD,
          new Constraints(
              Constants.GroundAlgae.MAX_ARM_VELOCITY, Constants.GroundAlgae.MAX_ARM_ACCELERATION));
  private final ArmFeedforward armFeedForward =
      new ArmFeedforward(
          Constants.GroundAlgae.sS,
          Constants.GroundAlgae.sG,
          Constants.GroundAlgae.sV,
          Constants.GroundAlgae.sA);
  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  private final PIDController pid =
      new PIDController(
          Constants.GroundAlgae.kP, Constants.GroundAlgae.kI, Constants.GroundAlgae.kD);

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          Constants.GroundAlgae.kS, Constants.GroundAlgae.kV, Constants.GroundAlgae.kA);

  public GroundAlgae(GroundAlgaeIO io) {
    this.io = io;

    toggle(GroundAlgaeStates.OFF);

    sysIdRoutineWheel =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state ->
                    Logger.recordOutput("GroundAlgae/Wheel/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setWheelVoltage(v.magnitude()), null, this));

    sysIdRoutineArm =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(3).per(Second),
                Volts.of(1.7),
                Seconds.of(1),
                (state -> Logger.recordOutput("GroundAlgae/Arm/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(v -> io.setArmVoltage(v.magnitude()), null, this));
  }

  private final SysIdRoutine sysIdRoutineWheel;
  private final SysIdRoutine sysIdRoutineArm;

  public enum GroundAlgaeStates {
    IN,
    OUT,
    OFF;
  }

  public enum GroundAlgaeArmStates {
    UP,
    DOWN,
    OFF;
  }

  public void toggle(GroundAlgaeStates state, double speed) {
    switch (state) {
      case IN:
        pid.setSetpoint(speed);
        break;
      case OUT:
        pid.setSetpoint(-speed);
        break;
      case OFF:
      default:
        pid.setSetpoint(0);
        break;
    }
  }

  public void toggle(GroundAlgaeStates state) {
    toggle(state, Constants.GroundAlgae.WHEEL_VELOCITY);
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return io.getWheelPosition();
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return io.getWheelVelocity() / 60.0 /* RPM -> RPS */;
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getArmPosition() / Constants.GroundAlgae.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getArmVelocity()
        / 60.0 /* RPM -> RPS */
        / Constants.GroundAlgae.ARM_GEAR_RATIO
        * 2
        * Math.PI;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getArmPercent() * RobotController.getBatteryVoltage();
  }

  public void toggleArm(double pos) {
    armPID.setGoal(pos);
  }

  public void toggleArm(GroundAlgaeArmStates state) {
    switch (state) {
      case UP:
        toggleArm(Constants.GroundAlgae.ARM_UP);
        break;
      case DOWN:
        toggleArm(Constants.GroundAlgae.ARM_DOWN);
        break;
      default:
        io.setArmVoltage(0);
        break;
    }
  }

  public boolean armAtGoal() {
    return armPID.atGoal();
  }

  @AutoLogOutput
  public boolean hasAlgae() {
    return debounced_value;
  }

  private double lastSpeed = 0;

  public void periodic() {
    debounced_value = !m_debouncer.calculate(io.getAlgaeDetector());

    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    lastSpeed = pid.getSetpoint();

    double pidOutputArm = armPID.calculate(getArmAngle());
    double armFeedforwardOutput =
        armFeedForward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, armPID.getSetpoint().velocity);
    io.setArmVoltage(pidOutputArm + armFeedforwardOutput);
    currentSetpoint = armPID.getSetpoint();
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
  public void kill() {
    toggle(GroundAlgaeStates.OFF);
    io.setWheelVoltage(0);
    io.setArmVoltage(0);
  }

  /** frees up all hardware allocations */
  @Override
  public void close() throws Exception {
    io.close();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Ground Algae Arm Pos", io.getArmPosition(), -1, 1)
        .subscribe(this::toggleArm);
    // TODO more shuffleboard stuff
    DashboardUI.Test.addSlider("Ground Algae", io.getWheelPercent(), -1, 1)
        .subscribe(io::setWheelPercent);
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getWheelCurrentDrawAmps() + io.getArmCurrentDrawAmps();
  }
}
