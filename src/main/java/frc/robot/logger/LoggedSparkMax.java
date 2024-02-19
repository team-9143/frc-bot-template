package frc.robot.logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** CANSparkMax wrapper class with logging functionality. */
public class LoggedSparkMax extends CANSparkMax implements Loggable {
  private static final String LOG_DIR = "/sparkmaxes/";
  public final String directory;

  public final RelativeEncoder encoder = getEncoder();

  private final double maxVoltage;

  /**
   * Create a new object to interface and log a SPARK MAX motor controller.
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected to their matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the Red and Black terminals only.
   * @param directory sub-directory where data will be logged (with trailing slash)
   * @param maxVoltage positive maximum voltage to provide to the motor
   * @param currentLimit SmartCurrentLimit to set for this motor controller (highly recommended)
   */
  public LoggedSparkMax(int deviceId, MotorType type, String directory, double maxVoltage, int currentLimit) {
    super(deviceId, type);

    this.maxVoltage = maxVoltage;

    setSmartCurrentLimit(currentLimit);
    encoder.setMeasurementPeriod(20); // Measurement period should be consistent with robot update period

    // Register for periodic logging
    this.directory = LOG_DIR+directory;
    Logger.registerLoggable(this);
  }

  /**
   * Clamps and sets the voltage output of the SpeedController. The behavior of this call differs slightly
   * from the WPILib documetation for this call since the device internally sets the desired voltage
   * (not a compensation value). That means that this *can* be a 'set-and-forget' call.
   *
   * @param outputVolts The voltage to output.
   */
  @Override
  public void setVoltage(double outputVolts) {
    super.setVoltage(Math.max(-maxVoltage, Math.min(maxVoltage, outputVolts)));
  }

  @Override
  public String getDirectory() {
    return directory;
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