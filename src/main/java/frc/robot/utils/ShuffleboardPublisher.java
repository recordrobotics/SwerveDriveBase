package frc.robot.utils;

public interface ShuffleboardPublisher {
  public void setupShuffleboard();

  public static void setup(ShuffleboardPublisher... publishers) {
    for (ShuffleboardPublisher publisher : publishers) {
      publisher.setupShuffleboard();
    }
  }
}
