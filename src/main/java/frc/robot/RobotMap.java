package frc.robot;

public final class RobotMap {
  public static final class Elevator {
    public static final int MOTOR_LEFT_ID = 9;
    public static final int MOTOR_RIGHT_ID = 10;
    public static final int BOTTOM_ENDSTOP_ID = -1; // TODO get actual DIO port number
    public static final int TOP_ENDSTOP_ID = -1; // TODO get actual DIO port number
  }

  public static final class CoralShooter {
    public static final int SERVO_ID = 0;
    public static final int MOTOR_ID = -1; // TODO get actual CAN number
    public static final int LIMIT_SWITCH_ID = -1; // TODO get actual DIO port number
  }

  public static final class Lights {
    public static final int LED_ID = -1; // TODO get actual CAN number
  }

  public static final class GroundAlgae {
    public static final int MOTOR_ID = -1; // TODO get actual port number
    public static final int LIMIT_SWITCH_ID = -1; // TODO get actual DIO port number
  }
}
