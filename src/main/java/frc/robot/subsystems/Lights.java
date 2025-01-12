package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  public enum LightMode {
    OFF,
    RUNNING,
    SUCCESS,
    FAIL
  }

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public Lights() {
    LEDs = new AddressableLED(-1); // TODO do port stuff
    buffer = new AddressableLEDBuffer(150); // TODO Constant
    LEDs.setLength(150); // TODO same Constant ^^^^^^^^^^^^^
    LEDs.start();
    setMode(LightMode.OFF);

    // Default command makes lights turn off by default
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  /**
   * NOTE: Only interact with light strip with the command!!!
   */
  public void setMode(LightMode mode) {
    switch (mode) {
      case OFF:
        runPattern(LEDPattern.solid(Color.kBlack));
        break;
      case RUNNING:
        LEDPattern pattern = LEDPattern.solid(Color.kOrange); // Just orange
        pattern = pattern.breathe(Seconds.of(1)); // Pulsating orange
        pattern = pattern.blend(LEDPattern.solid(Color.kOrange)); // Prevent from going to 0
        break;
      case SUCCESS:
        runPattern(LEDPattern.solid(Color.kGreen));
        break;
      case FAIL:
        runPattern(LEDPattern.solid(Color.kRed));
        break;
    }
  }

  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    LEDs.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }
}
