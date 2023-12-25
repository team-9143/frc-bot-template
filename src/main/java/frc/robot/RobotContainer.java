package frc.robot;

import frc.robot.util.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import frc.robot.devices.OI;
import frc.robot.devices.Controller.btn;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import frc.robot.util.SafeSubsystem;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
    Logger.recordMetadata("RoborioSerialNum", RobotBase.isReal() ? System.getenv("serialnum") : "Simulation");
    Logger.recordMetadata("BuildDate", LocalDateTime.now(ZoneId.of("UTC-8")).format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));

    // Close metadata acceptance and open output logging
    Logger.start();
  }

  /** Initialize OI devices. */
  private static void configureOI() {
    DriverStation.silenceJoystickConnectionWarning(true); // Stop those ridiculously persistent messages
  }

  /** Create button bindings. */
  private static void configureBindings() {
    // Button 'B' (hold) will continuously stop all movement
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.B) || OI.OPERATOR_CONTROLLER.getButton(btn.B))
      .whileTrue(new RunCommand(
        RobotContainer::stop,
        SafeSubsystem.getAll() // Requires all subsystems
      ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));// Interrupt incoming commands to ensure stop command takes precedence

    configureDriver();
    configureOperator();
  }

  private static void configureDriver() {
    // Button 'X' (debounced 0.5s) will reset gyro
    final var cRumble = OI.DRIVER_CONTROLLER.getRumbleCommand(0.5, 0.5, 0.25);
    new Trigger(() -> OI.DRIVER_CONTROLLER.getButton(btn.X))
    .debounce(0.3) // Wait 0.3s to avoid accidental press
      .onTrue(new InstantCommand(() -> {
        OI.PIGEON2.setYaw(0); // Reset gyro
        cRumble.schedule(); // Rumble to indicate event
      }));

    // Button 'Y' (hold) will set drivetrain to x-stance (for stability)
    final var cXStance = new RunCommand(Drivetrain.getInstance()::toXStance, Drivetrain.getInstance());
    OI.DRIVER_CONTROLLER.onTrue(btn.Y, cXStance::schedule);
    OI.DRIVER_CONTROLLER.onFalse(btn.Y, cXStance::cancel);
  }

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