package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

/** Subsystem with a stop method for safety measures. */
public abstract class SafeSubsystem extends SubsystemBase {
  private static final ArrayList<SafeSubsystem> s_subsystems = new ArrayList<>();

  public SafeSubsystem() {
    s_subsystems.add(this);
  }

  /** @return a list of all initalized subsystems */
  public static SafeSubsystem[] getAll() {
    return s_subsystems.toArray(SafeSubsystem[]::new);
  }

  /** Safe state method. Should be called in emergencies and when disabled. */
  public abstract void stop();
}