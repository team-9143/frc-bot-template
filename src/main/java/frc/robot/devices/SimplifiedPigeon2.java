package frc.robot.devices;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * This class is meant to remove much of the functionality of the Pigeon2 so that data cannot be set by unauthorized code. If new data needs to be available, wrapper methods may be added easily. This class does not implement the Loggable interface because IMU logging should be done by the subsystem it is associated with (usually, the drivetrain). Additionally, the constructor is able to clearly set the orientation of the IMU for standardized use across robots.
 * <p> Roll should increase to robot-right, pitch to robot-front, and yaw counter-clockwise. </p>
*/
public class SimplifiedPigeon2 {
  private final Pigeon2 PIGEON2;

  private final boolean invertPitch;
  private final boolean invertRoll;

  private final boolean swapPitchAndRoll;

  /**
   * Creates a SimplifiedPigeon2 with specified offsets and inversions to ensure consistent units and directions.
   *
   * @param deviceNumber CAN Device ID of the Pigeon
   * @param pitchOffset Additive pitch offset, best found by calibrating through PhoenixTuner
   * @param rollOffset Additive roll offset, best found by calibrating through PhoenixTuner
   * @param invertPitch Whether to invert the pitch to reach the proper positive direction (robot-front)
   * @param invertRoll Whether to invert the roll to reach the proper positive direction (robot-right)
   * @param swapPitchAndRoll Whether to swap the pitch (should be forward/backward) and the roll (should be left/right) to align the axes
   */
  protected SimplifiedPigeon2(int deviceNumber, double pitchOffset, double rollOffset, boolean invertPitch, boolean invertRoll, boolean swapPitchAndRoll) {
    PIGEON2 = new Pigeon2(deviceNumber);
    this.invertPitch = invertPitch;
    this.invertRoll = invertRoll;
    this.swapPitchAndRoll = swapPitchAndRoll;

    // Set up the mount pose and reset the yaw
    //PIGEON2.configMountPose(0, pitchOffset, rollOffset); HMMMMM
    PIGEON2.setYaw(0);
  }

  /**
   * Creates a SimplifiedPigeon2 with specified offsets. Assumes no inversions are necessary, e.g. where Pigeon is flat with the X-axis toward the front of the robot.
   *
   * @param deviceNumber CAN Device ID of the Pigeon
   * @param pitchOffset Additive pitch offset, best found by calibrating through PhoenixTuner
   * @param rollOffset Additive roll offset, best found by calibrating through PhoenixTuner
   */
  protected SimplifiedPigeon2(int deviceNumber, double pitchOffset, double rollOffset) {
    this(deviceNumber, pitchOffset, rollOffset, false, false, false);
  }

  /** @return IMU yaw, counter-clockwise positive. If this IMU is being used for odometry, use the odometry heading instead of this method to avoid conflicting data due to odometry pose resets */
  public double getYaw() {
    return PIGEON2.getYaw().getValueAsDouble();
  }

  /** @return IMU pitch, robot-front positive */
  public double getPitch() {
    if (swapPitchAndRoll) {
      return invertPitch ? PIGEON2.getRoll().getValueAsDouble() * -1 : PIGEON2.getRoll().getValueAsDouble(); // Invert the role if needed
    }
    return invertPitch ? PIGEON2.getPitch().getValueAsDouble() * -1 : PIGEON2.getPitch().getValueAsDouble();
  }

  /** @return IMU roll, robot-right positive */
  public double getRoll() {
    if (swapPitchAndRoll) {
      return invertRoll ? PIGEON2.getPitch().getValueAsDouble() * -1 : PIGEON2.getPitch().getValueAsDouble();
    }
    return invertRoll ? PIGEON2.getRoll().getValueAsDouble() * -1 : PIGEON2.getRoll().getValueAsDouble();
  }
}