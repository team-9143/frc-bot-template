package frc.robot.logger;

import com.ctre.phoenix6.hardware.CANcoder;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** CANCoder interface class with logging functionality. Many configuration parameters have been removed for constant units, but an offset has been added. */
public class LoggedCANcoder implements Loggable {
  private static final String LOG_DIR = "/cancoders/";
  public final String directory;

  /** CANcoder configuration options */
  private static final CANcoderConfiguration config = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

  private final CANcoder cancoder;

  private final Supplier<Double> positionSupplier;
  private final Supplier<Double> velocitySupplier;
  private final Supplier<Double> travelSupplier;

  private double offset;

  /**
   * Create a new object to interface and log a CANCoder absolute magnetic encoder.
   *
   * @param deviceId The device ID.
   * @param directory sub-directory where data will be logged (with trailing slash)
   */
  public LoggedCANcoder(int deviceId, String directory) {
    // Configure cancoder
    cancoder = new CANcoder(deviceId);
    cancoder.getConfigurator().apply(config);

    // TODO: Do this on a case-by-case basis or simplify this object to only be used for swerve
    // Configure status signals
    var positionSignal = cancoder.getAbsolutePosition();
    positionSignal.setUpdateFrequency(50);
    positionSupplier = positionSignal.asSupplier();

    var velocitySignal = cancoder.getVelocity();
    velocitySignal.setUpdateFrequency(50);
    velocitySupplier = velocitySignal.asSupplier();

    var travelSignal = cancoder.getPositionSinceBoot();
    travelSignal.setUpdateFrequency(50);
    travelSupplier = travelSignal.asSupplier();

    // Remove all other automatic updates
    cancoder.optimizeBusUtilization();

    // Register for periodic logging
    this.directory = LOG_DIR+directory;
    Logger.registerLoggable(this);
  }

  /**
   * Create a new object to interface and log a CANCoder absolute magnetic encoder.
   *
   * @param deviceId The device ID.
   * @param directory sub-directory where data will be logged (with trailing slash)
   * @param offset additive offset for {@link LoggedCANCoder#getPosition()} in ccw degrees
   */
  public LoggedCANcoder(int deviceId, String directory, double offset) {
    this(deviceId, directory);
    this.offset = offset;
  }

  /**
   * Get the absolute position of the sensor, which remains constant through a power cycle. Unaffected by {@link LoggedCANcoder#setOffset setOffset()} and {@link LoggedCANcoder#setPosition setPosition()}. UNIT: ccw degrees, range [0, 360)
   *
   * @return The position of the sensor.
   */
  public double getAbsolutePosition() {
    return positionSupplier.get() * 360d;
  }

  /**
   * Set the additive offset for {@link LoggedCANcoder#getOffsetPosition getOffsetPosition()}. UNIT: ccw degrees
   *
   * @param offset the offset
   */
  public void setOffset(double offset) {
    this.offset = offset;
  }

  /**
   * Get the offset position of the sensor, which remains constant through a power cycle. Affected by {@link LoggedCANcoder#setOffset setOffset()} and {@link LoggedCANcoder#setPosition setPosition()}. UNIT: ccw degrees, range [0, 360)
   *
   * @return The offset position of the sensor.
   */
  public double getOffsetPosition() {
    // Add the offset and then ensure the range binding remains stable
    return (getAbsolutePosition() + offset) % 360;
  }

  /**
   * Get the cumulative angle of the sensor, which resets to 0 on a power cycle. Unaffected by the position offset. UNIT: ccw degrees
   *
   * @return The distance of the sensor.
   */
  public double getTravel() {
    return travelSupplier.get() * 360d;
  }

  /**
   * Get the velocity of the sensor. UNIT: ccw degrees/second
   *
   * @return The velocity of the sensor.
   */
  public double getVelocity() {
    return velocitySupplier.get() * 360d;
  }

  @Override
  public String getDirectory() {
    return directory;
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"offsetDegCCW", this.getOffsetPosition());
    // Logger.recordOutput(getDirectory()+"velocityDegPerSecondCCW", this.getVelocity());
    Logger.recordOutput(getDirectory()+"totalDegCCW", this.getTravel());
  }
}