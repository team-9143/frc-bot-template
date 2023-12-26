package frc.robot.logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** CANSparkMax wrapper class with logging functionality through the Loggable interface. */
public class LoggedSparkMax extends CANSparkMax implements Loggable {
  private static final String LOG_DIR = "/motors/";
  public final String directory;

  public final RelativeEncoder encoder = getEncoder();

  /**
   * Create a new object to control and log a SPARK MAX motor controller.
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the Red and Black terminals only.
   * @param directory sub-directory where data will be logged (with trailing slash)
   */
  public LoggedSparkMax(int deviceId, MotorType type, String directory) {
    super(deviceId, type);
    this.directory = directory;

    // Register for periodic logging
    Logger.registerLoggable(this);
  }

  @Override
  public String getDirectory() {
    return LOG_DIR+directory;
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"percentOut", this.get());
    Logger.recordOutput(getDirectory()+"currentAmps", this.getOutputCurrent());
    Logger.recordOutput(getDirectory()+"tempCelsius", this.getMotorTemperature());
    Logger.recordOutput(getDirectory()+"speedRPM", encoder.getVelocity() / encoder.getVelocityConversionFactor());
    Logger.recordOutput(getDirectory()+"rotations", encoder.getPosition() / encoder.getPositionConversionFactor());
  }
}