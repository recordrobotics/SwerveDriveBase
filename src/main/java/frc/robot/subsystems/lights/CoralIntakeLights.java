package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.Lights;

public class CoralIntakeLights extends VirtualLightsSubsystem {

  public CoralIntakeLights(Lights lights) {
    super(lights, 30, 45);
    setDefaultCommand(runPattern(LEDPattern.kOff).ignoringDisable(true));
  }
}
