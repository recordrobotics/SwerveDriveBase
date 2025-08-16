package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMove;

public class ElevatorReefToggled extends Command {

    private ElevatorHeight targetHeight;
    private Command lightsCommand;

    public ElevatorReefToggled(ElevatorHeight targetHeight) {
        this.targetHeight = targetHeight;
        addRequirements(ToggleRequirements.elevatorMoveToggleRequirement);
    }

    @Override
    public void initialize() {
        lightsCommand = RobotContainer.lights.elevator.runPattern(Constants.Lights.elevatorPattern);
        lightsCommand.schedule();
        new ElevatorMove(targetHeight).handleInterrupt(this::cancel).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        new ElevatorMove(ElevatorHeight.BOTTOM).finallyDo(lightsCommand::cancel).schedule();
    }
}
