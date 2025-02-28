package frc.robot.commands.simulation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import java.util.HashSet;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AlgaeGrabberSim extends SequentialCommandGroup implements SimulationCommand {

  public enum AlgaeGrabberType {
    FAKE_TELEOP,
    FIELD_SETUP
  }

  private static LoggedDashboardChooser<AlgaeGrabberType> algaeGrabberTypeChooser =
      new LoggedDashboardChooser<>("Simulation/AlgaeGrabberType");

  public AlgaeGrabberSim(double secondsUntilAlgaeAcquired) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    algaeGrabberTypeChooser.addDefaultOption("Fake Teleop", AlgaeGrabberType.FAKE_TELEOP);
    algaeGrabberTypeChooser.addOption("Field Setup", AlgaeGrabberType.FIELD_SETUP);

    addCommands(
        new DeferredCommand(
            () -> {
              switch (algaeGrabberTypeChooser.get()) {
                case FAKE_TELEOP:
                  if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE) {
                    return new AlgaeGrabberFromGroundTimeBased(secondsUntilAlgaeAcquired);
                  } else {
                    return new AlgaeGrabberFromReef();
                  }
                case FIELD_SETUP:
                  if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE) {
                    return new AlgaeGrabberFromGroundPoseBased(RobotContainer.model::getAlgaes);
                  } else {
                    return new AlgaeGrabberFromReef();
                  }
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
