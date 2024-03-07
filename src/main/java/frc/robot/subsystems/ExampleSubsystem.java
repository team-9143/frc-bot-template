package frc.robot.subsystems;

// TODO(user): Remove
/** An example subsystem that serves no purpose whatsoever. */
public class ExampleSubsystem extends SafeSubsystem {
  private static final ExampleSubsystem m_instance = new ExampleSubsystem();

  /** Returns the singleton instance */
  public static ExampleSubsystem getInstance() {
    return m_instance;
  }

  /** Initialization (e.g. motors and default command). */
  private ExampleSubsystem() {}

  /** Methods that should be run every robot loop (e.g. updates and calculations). */
  @Override
  public void periodic() {}

  @Override
  public void log() {}

  @Override
  public void stop() {}
}
