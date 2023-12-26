package frc.robot.logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** CANSparkMax wrapper class with logging functionality through the Loggable interface. */
public class LoggedSparkMax extends CANSparkMax implements Loggable {
  private static final String LOG_DIR = "/motors/";
  public final String name;

  public final RelativeEncoder encoder = getEncoder();

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

    // Register for periodic logging
    Logger.registerLoggable(this);
  }

  @Override
  public String getDirectory() {
    return LOG_DIR+name;
  }

  @Override
  public void log() {
    Logger.recordOutput(LOG_DIR+name+"/percentOut", this.get());
    Logger.recordOutput(LOG_DIR+name+"/currentAmps", this.getOutputCurrent());
    Logger.recordOutput(LOG_DIR+name+"/tempCelsius", this.getMotorTemperature());
    Logger.recordOutput(LOG_DIR+name+"/speedRPM", encoder.getVelocity() / encoder.getVelocityConversionFactor());
    Logger.recordOutput(LOG_DIR+name+"/rotations", encoder.getPosition() / encoder.getPositionConversionFactor());
  }
}