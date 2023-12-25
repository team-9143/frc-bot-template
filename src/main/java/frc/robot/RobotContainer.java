package frc.robot;

import frc.robot.util.Logger;

import frc.robot.devices.OI;
import frc.robot.devices.Controller.btn;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.util.SafeSubsystem;
import frc.robot.subsystems.*;

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

    configureMetadata();
    configureOI();
    configureBindings();
  }

  /** Send metadata to logger, then start logger. */
  private static void configureMetadata() {
    Logger.recordMetadata("RoborioSerialNumber", System.getenv("serialnum"));

    // Close metadata acceptance and open output logging
    Logger.start();
  }

  /** Initialize OI devices. */
  private static void configureOI() {}

  /** Create button bindings. */
  private static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
      .whileTrue(new RunCommand(
        RobotContainer::stop,
        SafeSubsystem.getAll() // All subsystems are requirements
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

  /** Calls all subsystem stop methods. Does not stop commands. */
  public static void stop() {
   for (SafeSubsystem e : SafeSubsystem.getAll()) {
    e.stop();
   }
  }
}