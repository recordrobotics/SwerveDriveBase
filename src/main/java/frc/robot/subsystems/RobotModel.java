package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Represents the physical model of the robot, including mechanisms and their positions */
public class RobotModel extends SubsystemBase {

  public interface RobotMechanism {
    int getPoseCount();

    void updatePoses(Pose3d[] poses, int i);
  }

  public static class Elevator implements RobotMechanism {
    public static final int POSE_COUNT = 2; /* 2 stage elevator */

    @AutoLogOutput
    private LoggedMechanism2d mechanism =
        new LoggedMechanism2d(Constants.Frame.BUMPER_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

    private LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "elevator_root",
            Constants.Elevator.ROOT_MECHANISM_POSE.getX() + Constants.Frame.BUMPER_WIDTH / 2.0,
            Constants.Elevator.ROOT_MECHANISM_POSE.getY());
    private LoggedMechanismLigament2d elevator =
        root.append(
            new LoggedMechanismLigament2d(
                "elevator", Constants.Elevator.MIN_LENGTH, 90, 10, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d coralShooter =
        elevator.append(
            new LoggedMechanismLigament2d(
                "coralShooter",
                Constants.CoralShooter.LENGTH,
                -90,
                10,
                new Color8Bit(Color.kGreen)));

    @AutoLogOutput
    private LoggedMechanism2d mechanism_setpoint =
        new LoggedMechanism2d(Constants.Frame.BUMPER_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

    private LoggedMechanismRoot2d root_setpoint =
        mechanism_setpoint.getRoot(
            "elevator_root",
            Constants.Elevator.ROOT_MECHANISM_POSE.getX() + Constants.Frame.BUMPER_WIDTH / 2.0,
            Constants.Elevator.ROOT_MECHANISM_POSE.getY());
    private LoggedMechanismLigament2d elevator_setpoint =
        root_setpoint.append(
            new LoggedMechanismLigament2d(
                "elevator",
                Constants.Elevator.MIN_LENGTH,
                90,
                10,
                new Color8Bit(Color.kBlueViolet)));
    private LoggedMechanismLigament2d coralShooter_setpoint =
        elevator_setpoint.append(
            new LoggedMechanismLigament2d(
                "coralShooter",
                Constants.CoralShooter.LENGTH,
                -90,
                10,
                new Color8Bit(Color.kGreenYellow)));

    public void update(double height) {
      elevator.setLength(Constants.Elevator.MIN_LENGTH + height);
    }

    public void updateSetpoint(double height) {
      elevator_setpoint.setLength(Constants.Elevator.MIN_LENGTH + height);
    }

    @Override
    public int getPoseCount() {
      return POSE_COUNT;
    }

    @Override
    public void updatePoses(Pose3d[] poses, int i) {
      // TODO: get formula for getting position of each stage based on overall height

      // First stage
      poses[i++] =
          new Pose3d(
              new Translation3d(0, 0, (1 + Math.cos(2 * Timer.getFPGATimestamp())) / 2.0 * 0.8),
              new Rotation3d(0, 0, 0));

      // Second stage
      poses[i++] =
          poses[i - 2].transformBy(
              new Transform3d(
                  0,
                  0,
                  (1 + Math.cos(2 * Timer.getFPGATimestamp())) / 2.0 * 0.675,
                  new Rotation3d(0, 0, 0)));
    }
  }

  public static class CoralIntake implements RobotMechanism {
    public static final int POSE_COUNT = 1;

    @AutoLogOutput
    private LoggedMechanism2d mechanism =
        new LoggedMechanism2d(Constants.Frame.BUMPER_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

    private LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "coralintake_root",
            Constants.CoralIntake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.BUMPER_WIDTH / 2.0,
            Constants.CoralIntake.ROOT_MECHANISM_POSE.getY());
    private LoggedMechanismLigament2d coralintake =
        root.append(
            new LoggedMechanismLigament2d(
                "coralintake",
                Constants.CoralIntake.LENGTH,
                Constants.CoralIntake.ANGLE_OFFSET,
                10,
                new Color8Bit(Color.kPurple)));

    @AutoLogOutput
    private LoggedMechanism2d mechanism_setpoint =
        new LoggedMechanism2d(Constants.Frame.BUMPER_WIDTH, Constants.Frame.MAX_MECHANISM_HEIGHT);

    private LoggedMechanismRoot2d root_setpoint =
        mechanism_setpoint.getRoot(
            "coralintake_root",
            Constants.CoralIntake.ROOT_MECHANISM_POSE.getX() + Constants.Frame.BUMPER_WIDTH / 2.0,
            Constants.CoralIntake.ROOT_MECHANISM_POSE.getY());
    private LoggedMechanismLigament2d coralintake_setpoint =
        root_setpoint.append(
            new LoggedMechanismLigament2d(
                "coralintake",
                Constants.CoralIntake.LENGTH,
                Constants.CoralIntake.ANGLE_OFFSET,
                10,
                new Color8Bit(Color.kViolet)));

    public void update(double angle) {
      coralintake.setAngle(Constants.CoralIntake.ANGLE_OFFSET + angle);
    }

    public void updateSetpoint(double angle) {
      coralintake_setpoint.setAngle(Constants.CoralIntake.ANGLE_OFFSET + angle);
    }

    @Override
    public int getPoseCount() {
      return POSE_COUNT;
    }

    @Override
    public void updatePoses(Pose3d[] poses, int i) {
      poses[i] =
          new Pose3d(0, 0, 0, new Rotation3d())
              .rotateAround(
                  new Translation3d(0, 0.334669, 0.456817),
                  new Rotation3d(
                      coralintake.getAngle() + Math.cos(2 * Timer.getFPGATimestamp()), 0, 0));
    }
  }

  public final Elevator elevator = new Elevator();
  public final CoralIntake coralIntake = new CoralIntake();

  @AutoLogOutput
  public Pose3d[] mechanismPoses = new Pose3d[Elevator.POSE_COUNT + CoralIntake.POSE_COUNT];

  @AutoLogOutput public Pose2d robot = new Pose2d();

  public RobotModel() {
    periodic();
  }

  @Override
  public void periodic() {
    updatePoses(elevator, coralIntake);
  }

  private void updatePoses(RobotMechanism... mechanisms) {
    int i = 0;
    for (RobotMechanism mechanism : mechanisms) {
      if (i >= mechanismPoses.length) {
        DriverStation.reportError("RobotModel.updatePoses: too many mechanisms", false);
        break;
      }

      mechanism.updatePoses(mechanismPoses, i);
      i += mechanism.getPoseCount();
    }
  }
}
