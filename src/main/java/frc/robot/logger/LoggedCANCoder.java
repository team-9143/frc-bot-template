package frc.robot.logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** CANCoder interface class with logging functionality. Many configuration parameters have been removed for constant units, but an offset has been added. */
public class LoggedCANCoder implements Loggable {
  private static final String LOG_DIR = "/cancoders/";
  public final String directory;

  private final CANCoder cancoder;

  private double offset;

  /**
   * Create a new object to interface and log a CANCoder absolute magnetic encoder.
   *
   * @param deviceId The device ID.
   * @param directory sub-directory where data will be logged (with trailing slash)
   */
  public LoggedCANCoder(int deviceId, String directory) {
    // Set up cancoder
    cancoder = new CANCoder(deviceId);
    cancoder.configSensorDirection(false); // Set counterclockwise
    cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); // Set output range [0,360)
    cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero); // Set position offset to start at 0
    cancoder.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms); // Measurement period should be consistent with robot update period

    // Register for periodic logging
    this.directory = directory;
    Logger.registerLoggable(this);
  }

  /**
   * Create a new object to interface and log a CANCoder absolute magnetic encoder.
   *
   * @param deviceId The device ID.
   * @param directory sub-directory where data will be logged (with trailing slash)
   * @param offset additive offset for {@link LoggedCANCoder#getPosition()} in ccw degrees
   */
  public LoggedCANCoder(int deviceId, String directory, double offset) {
    this(deviceId, directory);
    this.offset = offset;
  }

  /**
   * Get the absolute position of the sensor, which remains constant through a power cycle. Unaffected by the position offset. UNIT: ccw degrees, range [0, 360)
   *
   * @return The position of the sensor.
   */
  public double getAbsolutePosition() {
    return cancoder.getAbsolutePosition();
  }

  /**
   * Set the additive offset. UNIT: ccw degrees
   *
   * @param offset the offset for {@link LoggedCANCoder#getPosition()}
   */
  public void setOffset(double offset) {
    this.offset = offset;
  }

  /**
   * Get the additive offset. UNIT: ccw degrees
   *
   * @return The offset for {@link LoggedCANCoder#getPosition()}
   */
  public double getOffset() {
    return offset;
  }

  /**
   * Get the offset position of the sensor, which remains constant through a power cycle. Affected by the position offset. UNIT: ccw degrees
   *
   * @return The offset position of the sensor.
   */
  public double getPosition() {
    return cancoder.getAbsolutePosition() + offset;
  }

  /**
   * Get the cumulative angle of the sensor, which resets to 0 on a power cycle. Unaffected by the position offset. UNIT: ccw degrees
   *
   * @return The distance of the sensor.
   */
  public double getTravel() {
    return cancoder.getPosition();
  }

  /**
   * Get the velocity of the sensor. UNIT: ccw degrees/second
   *
   * @return The velocity of the sensor.
   */
  public double getVelocity() {
    return cancoder.getVelocity();
  }

  @Override
  public String getDirectory() {
    return LOG_DIR+directory;
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"absoluteDegCCW", this.getAbsolutePosition());
    Logger.recordOutput(getDirectory()+"offsetDegCCW", this.getPosition());
    Logger.recordOutput(getDirectory()+"velocityDegPerSecondCCW", this.getVelocity());
    Logger.recordOutput(getDirectory()+"totalDegCCW", this.getTravel());
  }
}