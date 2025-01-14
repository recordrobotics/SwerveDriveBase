package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;

public class MacroPad extends GenericHID {

  public enum Button {
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    private final int value;

    Button(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public MacroPad(int port) {
    super(port);
  }

  public boolean getButton(Button button) {
    return getRawButton(button.getValue());
  }

  public boolean getLevel1Button() {
    return getButton(Button.L1);
  }

  public boolean getLevel2Button() {
    return getButton(Button.L2);
  }

  public boolean getLevel3Button() {
    return getButton(Button.L3);
  }

  public boolean getLevel4Button() {
    return getButton(Button.L4);
  }
}
