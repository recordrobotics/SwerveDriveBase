package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends KillableSubsystem implements ShuffleboardPublisher, PoweredSubsystem {
  private final ClimberIO io;
  private double targetPos = Constants.Climber.START_POS;

  public Climber(ClimberIO io) {
    this.io = io;

    io.applyTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setPosition(
        Constants.Climber.GEAR_RATIO * Units.radiansToRotations(Constants.Climber.START_POS));
  }

  @Override
  public void periodic() {
    double currentPos = getArmAngle();

    // Simple bang bang with deadband
    if (Math.abs(targetPos - currentPos) < Constants.Climber.DEADBAND) {
      io.setVoltage(0.0);
    } else if (targetPos > currentPos) {
      io.setVoltage(Constants.Climber.CLIMB_VOLTAGE);
    } else {
      io.setVoltage(-Constants.Climber.EXTEND_VOLTAGE);
    }

    // Update mechanism
    RobotContainer.model.climber.update(getArmAngle());
  }

  public boolean atGoal() {
    return Math.abs(targetPos - getArmAngle()) < Constants.Climber.DEADBAND;
  }

  public void climb() {
    targetPos = Constants.Climber.RETRACTED_POS;
  }

  public void extend() {
    targetPos = Constants.Climber.EXTENDED_POS;
  }

  public void toggle() {
    if (targetPos == Constants.Climber.EXTENDED_POS) {
      climb();
    } else {
      extend();
    }
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getPosition() / Constants.Climber.GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getVelocity() / Constants.Climber.GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getPercent() * RobotController.getBatteryVoltage();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public ClimberSim getSimIO() throws Exception {
    if (io instanceof ClimberSim) {
      return (ClimberSim) io;
    } else {
      throw new Exception("ClimberIO is not a simulation");
    }
  }

  @Override
  public void setupShuffleboard() {} // TODO add shuffleboard?

  @Override
  public void kill() {
    io.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getCurrentDrawAmps();
  }
}
