package frc.robot.utils;

public interface ShuffleboardPublisher {
    void setupShuffleboard();

    static void setup(ShuffleboardPublisher... publishers) {
        for (ShuffleboardPublisher publisher : publishers) {
            publisher.setupShuffleboard();
        }
    }
}
