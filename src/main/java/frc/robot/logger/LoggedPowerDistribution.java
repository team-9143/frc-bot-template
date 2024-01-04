package frc.robot.logger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.hal.PowerDistributionFaults;

public class LoggedPowerDistribution extends PowerDistribution implements Loggable {
  private static final String LOG_DIR = "/power/";

  // Faults to compare against to log differences
  private PowerDistributionFaults prevFaults = new PowerDistributionFaults(0);

  /**
   * Detects the connected PDP/PDH using the default CAN ID (0 for CTRE and 1 for REV).
   */
  public LoggedPowerDistribution() {
    super();

    // Register for periodic logging
    Logger.registerLoggable(this);

    // Initialize all faults in the log file
    logAllFaults(new PowerDistributionFaults(0));
  }

  @Override
  public String getDirectory() {
    return LOG_DIR;
  }

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"inputVoltage", this.getVoltage());

    // Log the output current for each channel on the power distribution device
    for (int i = 0; i < this.getNumChannels(); i++) {
      Logger.recordOutput(getDirectory()+"outputCurrent/"+i, this.getCurrent(i));
    }

    // Log any new faults
    logNewFaults();
  }

  private void logAllFaults(PowerDistributionFaults faults) {
    Logger.recordOutput(getDirectory()+"brownout", faults.Brownout);
    Logger.recordOutput(getDirectory()+"CANWarning", faults.CanWarning);
    Logger.recordOutput(getDirectory()+"hardwareFault", faults.HardwareFault);

    Logger.recordOutput(getDirectory()+"breakers/0", faults.Channel0BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/1", faults.Channel1BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/2", faults.Channel2BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/3", faults.Channel3BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/4", faults.Channel4BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/5", faults.Channel5BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/6", faults.Channel6BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/7", faults.Channel7BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/8", faults.Channel8BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/9", faults.Channel9BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/10", faults.Channel10BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/11", faults.Channel11BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/12", faults.Channel12BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/13", faults.Channel13BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/14", faults.Channel14BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/15", faults.Channel15BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/16", faults.Channel16BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/17", faults.Channel17BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/18", faults.Channel18BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/19", faults.Channel19BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/20", faults.Channel20BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/21", faults.Channel21BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/22", faults.Channel22BreakerFault);
    Logger.recordOutput(getDirectory()+"breakers/23", faults.Channel23BreakerFault);
  }

  /** Log any changes in faults since the last call */
  private void logNewFaults() {
    PowerDistributionFaults currFaults = this.getFaults();

    logIfDifferent(getDirectory()+"brownout", prevFaults.Brownout, currFaults.Brownout);
    logIfDifferent(getDirectory()+"CANWarning", prevFaults.CanWarning, currFaults.CanWarning);
    logIfDifferent(getDirectory()+"hardwareFault", prevFaults.HardwareFault, currFaults.HardwareFault);

    logIfDifferent(getDirectory()+"breakers/0", prevFaults.Channel0BreakerFault, currFaults.Channel0BreakerFault);
    logIfDifferent(getDirectory()+"breakers/1", prevFaults.Channel1BreakerFault, currFaults.Channel1BreakerFault);
    logIfDifferent(getDirectory()+"breakers/2", prevFaults.Channel2BreakerFault, currFaults.Channel2BreakerFault);
    logIfDifferent(getDirectory()+"breakers/3", prevFaults.Channel3BreakerFault, currFaults.Channel3BreakerFault);
    logIfDifferent(getDirectory()+"breakers/4", prevFaults.Channel4BreakerFault, currFaults.Channel4BreakerFault);
    logIfDifferent(getDirectory()+"breakers/5", prevFaults.Channel5BreakerFault, currFaults.Channel5BreakerFault);
    logIfDifferent(getDirectory()+"breakers/6", prevFaults.Channel6BreakerFault, currFaults.Channel6BreakerFault);
    logIfDifferent(getDirectory()+"breakers/7", prevFaults.Channel7BreakerFault, currFaults.Channel7BreakerFault);
    logIfDifferent(getDirectory()+"breakers/8", prevFaults.Channel8BreakerFault, currFaults.Channel8BreakerFault);
    logIfDifferent(getDirectory()+"breakers/9", prevFaults.Channel9BreakerFault, currFaults.Channel9BreakerFault);
    logIfDifferent(getDirectory()+"breakers/10", prevFaults.Channel10BreakerFault, currFaults.Channel10BreakerFault);
    logIfDifferent(getDirectory()+"breakers/11", prevFaults.Channel11BreakerFault, currFaults.Channel11BreakerFault);
    logIfDifferent(getDirectory()+"breakers/12", prevFaults.Channel12BreakerFault, currFaults.Channel12BreakerFault);
    logIfDifferent(getDirectory()+"breakers/13", prevFaults.Channel13BreakerFault, currFaults.Channel13BreakerFault);
    logIfDifferent(getDirectory()+"breakers/14", prevFaults.Channel14BreakerFault, currFaults.Channel14BreakerFault);
    logIfDifferent(getDirectory()+"breakers/15", prevFaults.Channel15BreakerFault, currFaults.Channel15BreakerFault);
    logIfDifferent(getDirectory()+"breakers/16", prevFaults.Channel16BreakerFault, currFaults.Channel16BreakerFault);
    logIfDifferent(getDirectory()+"breakers/17", prevFaults.Channel17BreakerFault, currFaults.Channel17BreakerFault);
    logIfDifferent(getDirectory()+"breakers/18", prevFaults.Channel18BreakerFault, currFaults.Channel18BreakerFault);
    logIfDifferent(getDirectory()+"breakers/19", prevFaults.Channel19BreakerFault, currFaults.Channel19BreakerFault);
    logIfDifferent(getDirectory()+"breakers/20", prevFaults.Channel20BreakerFault, currFaults.Channel20BreakerFault);
    logIfDifferent(getDirectory()+"breakers/21", prevFaults.Channel21BreakerFault, currFaults.Channel21BreakerFault);
    logIfDifferent(getDirectory()+"breakers/22", prevFaults.Channel22BreakerFault, currFaults.Channel22BreakerFault);
    logIfDifferent(getDirectory()+"breakers/23", prevFaults.Channel23BreakerFault, currFaults.Channel23BreakerFault);

    prevFaults = currFaults;
  }

  private void logIfDifferent(String path, boolean prev, boolean curr) {
    if (prev != curr) {
      Logger.recordOutput(path, curr);
    }
  }
}