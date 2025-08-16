package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.commands.ElevatorMoveThenAlgaeGrabEnd;

public class ElevatorAlgaeToggled extends Command {

    private ElevatorHeight targetHeight;

    public ElevatorAlgaeToggled(ElevatorHeight targetHeight) {
        this.targetHeight = targetHeight;
        addRequirements(ToggleRequirements.elevatorMoveToggleRequirement);
    }

    @Override
    public void initialize() {
        ElevatorMoveThenAlgaeGrab.create(targetHeight, false)
                .handleInterrupt(this::cancel)
                .schedule();
    }

    @Override
    public void end(boolean interrupted) {
        new ElevatorMoveThenAlgaeGrabEnd(targetHeight, true)
                .handleInterrupt(this::cancel)
                .schedule();
    }
}
