package frc.robot.commands.simulation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import java.util.HashSet;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CoralIntakeFromGroundSim extends SequentialCommandGroup implements SimulationCommand {

  public enum CoralIntakeType {
    TIME_BASED,
    POSE_BASED
  }

  private static LoggedDashboardChooser<CoralIntakeType> coralIntakeTypeChooser =
      new LoggedDashboardChooser<>("Simulation/CoralIntakeType");

  public CoralIntakeFromGroundSim(double secondsUntilCoralAcquired) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    coralIntakeTypeChooser.addDefaultOption("Time Based", CoralIntakeType.TIME_BASED);
    coralIntakeTypeChooser.addOption("Pose Based", CoralIntakeType.POSE_BASED);

    addCommands(
        new DeferredCommand(
            () -> {
              switch (coralIntakeTypeChooser.get()) {
                case TIME_BASED:
                  return new CoralIntakeFromGroundTimeBased(secondsUntilCoralAcquired);
                case POSE_BASED:
                  return new CoralIntakeFromGroundPoseBased(RobotContainer.model::getCorals);
                default:
                  return null;
              }
            },
            new HashSet<>()));
  }

  @Override
  public Command simulateFor(Command command) {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      return command;
    } else {
      return command.alongWith(this);
    }
  }
}
