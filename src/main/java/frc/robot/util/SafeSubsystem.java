package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem with a stop method for safety measures. */
public abstract class SafeSubsystem extends SubsystemBase {
  /** Safe state method. Should be called in emergencies and when disabled. */
  public abstract void stop();
}
