package frc.robot;

import frc.robot.devices.OI;
import frc.robot.devices.Controller.btn;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.RunCommand;

/** Robot structure declaration. Initializes trigger mappings, OI devices, and main stop mechanism. */
public class RobotContainer {
  private static RobotContainer m_instance;

  /** @return the singleton instance */
  public static RobotContainer getInstance() {
    if (m_instance == null) {
      m_instance = new RobotContainer();
    }
    return m_instance;
  }

  private RobotContainer() {
    // Configure pigeon - make sure to update pitch and roll offsets
    OI.PIGEON2.configMountPose(0, 0, 0); // TODO: Add pigeon pitch and roll offsets here
    OI.PIGEON2.setYaw(0);

    configureBindings();
  }

  /** Initialize button bindings. */
  private void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
      .whileTrue(new RunCommand(
        RobotContainer::stop
        // TODO: Add all subsystems as requirements here
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

  private void configureDriver() {}

  private void configureOperator() {}

  /** Stops all motors and disables controllers. Does not stop commands. */
  public static void stop() {
    // TODO: Add all subsystem stop methods here
  }
}