package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotMap;
import frc.robot.utils.KillableSubsystem;

public class Climber extends KillableSubsystem {
    private TalonFX motor;
    public boolean going = false;

    public Climber() {
        motor = new TalonFX(RobotMap.Climber.MOTOR_ID);
        motor.setVoltage(0);
    }

    public void periodic() {
        if (going) {
            motor.setVoltage(12);
        } else {
            motor.setVoltage(0);
        }
    }

    public void kill() {
        going = false;
        motor.setVoltage(0);
    }

    public void close() {
        kill();
        motor.close();
    }
}
