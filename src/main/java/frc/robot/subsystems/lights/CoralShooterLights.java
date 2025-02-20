package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.Lights;

public class CoralShooterLights extends VirtualLightsSubsystem {

  public CoralShooterLights(Lights lights) {
    super(lights, 16, 20);
    setDefaultCommand(runPattern(LEDPattern.kOff).ignoringDisable(true));
  }
}
