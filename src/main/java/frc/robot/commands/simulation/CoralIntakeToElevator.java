package frc.robot.commands.simulation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;

public class CoralIntakeToElevator extends SequentialCommandGroup implements SimulationCommand {

  private NamedCoral coral;

  public CoralIntakeToElevator() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(
            () -> coral = RobotContainer.model.getCoral("CoralIntakeToElevator/Coral")),
        // set does not have coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralIntake.getSimIO().setCoralDetectorSim(true);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }),
        // move coral to elevator
        new WaitCommand(0.4),
        new InstantCommand(() -> RobotContainer.model.removeCoral(coral)),
        // set has elevator coral (NC)
        new InstantCommand(
            () -> {
              try {
                RobotContainer.coralShooter.getSimIO().setCoralDetectorSim(false);
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
