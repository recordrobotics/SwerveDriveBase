package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class KillableSubsystem extends SubsystemBase {
  public abstract void kill();

  public abstract void close();
}
