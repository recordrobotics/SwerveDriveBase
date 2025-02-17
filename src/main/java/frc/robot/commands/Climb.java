package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Lights.LightSegments;

public class Climb extends SequentialCommandGroup {
  public Climb() {
    addCommands(
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.cagePattern),
        // TODO climbers
        new LightsCommand(LightSegments.STATE_VISUALIZER, Constants.Lights.OFF),
        new SuccessfulCompletion(true, true, true, true, true));
  }
}
