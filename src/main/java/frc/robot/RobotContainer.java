package frc.robot;

import frc.robot.devices.OI;
import frc.robot.devices.Controller.btn;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExampleStateSubsystem;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/** Robot structure declaration. Initializes trigger mappings, OI devices, and main stop mechanism. */
public class RobotContainer {
  private static boolean m_initialized = false;

  /** Initialize robot container. */
  public static void init() {
    if (m_initialized == true) {
      return;
    }
    m_initialized = true;

    configureOI();
    configureBindings();
  }

  /** Initialize OI devices. */
  private static void configureOI() {
    // Configure pigeon - make sure to update pitch and roll offsets
    OI.PIGEON2.configMountPose(0, 0, 0); // TODO: Add pigeon pitch and roll offsets here
    OI.PIGEON2.setYaw(0);
  }

  /** Create button bindings. */
  private static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
      .whileTrue(new RunCommand(
        RobotContainer::stop,
        ExampleSubsystem.getInstance(),
        ExampleStateSubsystem.getInstance()
      ) {
        @Override
        public InterruptionBehavior getInterruptionBehavior() {
          // Interrupt incoming commands to ensure stop command takes precedence
          return InterruptionBehavior.kCancelIncoming;
        }
      });

    configureDriver();
    configureOperator();
  }

  private static void configureDriver() {}

  private static void configureOperator() {
    // Button 'A' (hold) sets subsystem to fast
    final var cFast = new StartEndCommand(
      () -> ExampleStateSubsystem.getInstance().setState(ExampleStateSubsystem.States.FAST),
      () -> ExampleStateSubsystem.getInstance().setState(ExampleStateSubsystem.States.STOP),
      ExampleStateSubsystem.getInstance()
    );
    OI.OPERATOR_CONTROLLER.onTrue(btn.A, cFast::schedule);
    OI.OPERATOR_CONTROLLER.onFalse(btn.A, cFast::cancel);
  }

  /** Stops all motors and disables controllers. Does not stop commands. */
  public static void stop() {
    ExampleSubsystem.getInstance().stop();
    ExampleStateSubsystem.getInstance().stop();
  }
}