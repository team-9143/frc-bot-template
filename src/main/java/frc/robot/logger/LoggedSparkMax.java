package frc.robot.logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.ArrayList;

/** CANSparkMax wrapper class with logging functionality. {@link LoggedSparkMax#updateAll()} must be called periodically for logging. */
public class LoggedSparkMax extends CANSparkMax {
  private static final String MOTOR_LOG_DIR = "/motors/";
  private static final ArrayList<LoggedSparkMax> s_instances = new ArrayList<>();

  public final String name;

  public final RelativeEncoder encoder = getEncoder();

  public static void updateAll() {
    s_instances.forEach(e -> e.update());
  }

  /**
   * Create a new object to control and log a SPARK MAX motor controller.
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the Red and Black terminals only.
   * @param name Name of the device for logging.
   */
  public LoggedSparkMax(int deviceId, MotorType type, String name) {
    super(deviceId, type);
    this.name = name;
    s_instances.add(this);

    // Log initialization of spark max with format: "Spark Max [deviceId] '[name]' initialized"
    Logger.log(String.format("Spark Max %d '%s' initialized", deviceId, name));
  }

  private void update() {
    Logger.recordOutput(MOTOR_LOG_DIR+name+"/percentOut", this.get());
    Logger.recordOutput(MOTOR_LOG_DIR+name+"/currentAmps", this.getOutputCurrent());
    Logger.recordOutput(MOTOR_LOG_DIR+name+"/tempCelsius", this.getMotorTemperature());
    Logger.recordOutput(MOTOR_LOG_DIR+name+"/speedRPM", encoder.getVelocity() / encoder.getVelocityConversionFactor());
    Logger.recordOutput(MOTOR_LOG_DIR+name+"/rotations", encoder.getPosition() / encoder.getPositionConversionFactor());
  }
}