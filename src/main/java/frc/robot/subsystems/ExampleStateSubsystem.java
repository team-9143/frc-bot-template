package frc.robot.subsystems;

import frc.robot.util.StateSubsystem;

public class ExampleStateSubsystem extends StateSubsystem<ExampleStateSubsystem.States> {
  private static ExampleStateSubsystem m_instance;

  /** @return the singleton instance */
  public static synchronized ExampleStateSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ExampleStateSubsystem();
      m_instance.stop();
    }
    return m_instance;
  }

  /** Initialization (e.g. motors and default command). */
  private ExampleStateSubsystem() {}

  /** Main logic method to run methods based on current state. */
  @Override
  public void periodic() {
    switch (getState()) {
      case FAST:
        System.out.println(getName() + " is fast");
      case SLOW:
        System.out.println(getName() + " is slow");
      default:
        stop();
    }
  }

  @Override
  public void stop() {}

  public enum States {
    FAST,
    SLOW,
    STOP
  }
}