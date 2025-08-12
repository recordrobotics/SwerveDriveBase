package frc.robot.subsystems.io;

public interface NavSensorIO extends AutoCloseable {

    void reset();

    void resetDisplacement();

    double getAngle();

    double getWorldLinearAccelX();

    double getWorldLinearAccelY();

    boolean isConnected();

    void simulationPeriodic();
}
