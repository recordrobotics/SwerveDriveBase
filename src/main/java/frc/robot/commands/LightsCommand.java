package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class LightsCommand extends SequentialCommandGroup {
  public LightsCommand(LightSegments segment, LEDPattern pattern) {
    this(segment, () -> pattern);
  }

  public LightsCommand(LightSegments segment, Supplier<LEDPattern> pattern) {
    addCommands(
        new InstantCommand(
            () -> {
              RobotContainer.lights.patterns.put(segment, pattern);
            }));
  }
}
