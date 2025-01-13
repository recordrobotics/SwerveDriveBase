package frc.robot;

public final class RobotMap {
  public static final class Elevator {
    public static int MOTOR_LEFT_ID = 9;
    public static int MOTOR_RIGHT_ID = 10;
    public static int BOTTOM_ENDSTOP_ID = -1; // TODO get actual DIO port number
    public static int TOP_ENDSTOP_ID = -1; // TODO get actual DIO port number
  }

  public static final class CoralShooter {
    public static int MOTOR_ID = -1; // TODO get actual CAN number
    public static int LIMIT_SWITCH_ID = -1; // TODO get actual DIO port number
  }

  public static final class Lights {
    public static int LED_ID = -1; // TODO get actual CAN number
  }
}
