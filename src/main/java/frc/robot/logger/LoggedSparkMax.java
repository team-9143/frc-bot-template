package frc.robot.logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// TODO Set smart current limit and move calls to setVelocity
/** CANSparkMax wrapper class with logging functionality. */
public class LoggedSparkMax extends CANSparkMax implements Loggable {
  private static final String LOG_DIR = "/sparkmaxes/";
  public final String directory;

  public final RelativeEncoder encoder = getEncoder();

  /**
   * Create a new object to interface and log a SPARK MAX motor controller.
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the Red and Black terminals only.
   * @param directory sub-directory where data will be logged (with trailing slash)
   */
  public LoggedSparkMax(int deviceId, MotorType type, String directory) {
    super(deviceId, type);
    encoder.setMeasurementPeriod(20); // Measurement period should be consistent with robot update period

    // Register for periodic logging
    this.directory = directory;
    Logger.registerLoggable(this);
  }

  @Override
  public String getDirectory() {
    return LOG_DIR+directory;
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"currentAmps", this.getOutputCurrent());
    Logger.recordOutput(getDirectory()+"percentOut", this.get());
    Logger.recordOutput(getDirectory()+"speedRPM", encoder.getVelocity() / encoder.getVelocityConversionFactor());
    Logger.recordOutput(getDirectory()+"totalRotations", encoder.getPosition() / encoder.getPositionConversionFactor());
    Logger.recordOutput(getDirectory()+"motorTempCelsius", this.getMotorTemperature());
  }
}