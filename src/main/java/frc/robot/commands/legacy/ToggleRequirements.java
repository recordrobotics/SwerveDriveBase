package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToggleRequirements {

    public static class ElevatorMoveToggleRequirement extends SubsystemBase {}

    public static ElevatorMoveToggleRequirement elevatorMoveToggleRequirement = new ElevatorMoveToggleRequirement();

    public static class CoralIntakeMoveToggleRequirement extends SubsystemBase {}

    public static CoralIntakeMoveToggleRequirement coralIntakeMoveToggleRequirement =
            new CoralIntakeMoveToggleRequirement();
}
