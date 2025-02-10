package frc.robot.commands.simulation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;

public class CoralIntakeFromGroundTimeBased extends SequentialCommandGroup
    implements SimulationCommand {

  public CoralIntakeFromGroundTimeBased(double secondsUntilCoralAcquired) {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    NamedCoral coral =
        new NamedCoral(
            "CoralIntakeFromGroundTimeBased/Coral",
            RobotContainer.model.coralIntake.getCoralTargetPose());
    addCommands(
        new WaitCommand(secondsUntilCoralAcquired),
        new InstantCommand(() -> RobotContainer.model.addCoral(coral)),
        // TODO: move coral to intake
        // mark coral for use for the elevator
        new InstantCommand(() -> coral.name = "CoralIntakeToElevator/Coral"),
        // set has coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralIntake.getSimIO().setCoralDetectorSim(false);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }));
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
