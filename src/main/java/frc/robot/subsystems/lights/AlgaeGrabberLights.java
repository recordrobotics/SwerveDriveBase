package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.Lights;

public class AlgaeGrabberLights extends VirtualLightsSubsystem {

  public AlgaeGrabberLights(Lights lights) {
    super(lights, 6, 10);
    setDefaultCommand(runPattern(LEDPattern.kOff).ignoringDisable(true));
  }
}
