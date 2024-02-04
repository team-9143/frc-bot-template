package frc.robot.devices;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.configs.MountPoseConfigs;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * This class implements configuration and angle measurement functionality of a Pigeon 2. It is meant mainly to be used to stop unauthorized resetting of this data, as it should only be changed by the odometry.
 * <p> Roll should increase to robot-right, pitch to robot-front, and yaw counter-clockwise. </p>
*/
public class SimplifiedPigeon2 {
  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> pitchSignal;
  private final StatusSignal<Double> rollSignal;

  private final Pigeon2 pigeon2;

  private final double invertPitch;
  private final double invertRoll;
  private final boolean swapPitchAndRoll;

  /**
   * Creates a SimplifiedPigeon2 with specified offsets and inversions to ensure consistent units and directions.
   *
   * @param deviceNumber CAN Device ID of the Pigeon
   * @param pitchOffset Additive pitch offset, best found by calibrating through PhoenixTuner
   * @param rollOffset Additive roll offset, best found by calibrating through PhoenixTuner
   * @param invertPitch Whether to invert the pitch to reach the proper positive direction (robot-front)
   * @param invertRoll Whether to invert the roll to reach the proper positive direction (robot-right)
   * @param swapPitchAndRoll Whether to swap the pitch (should be forward/backward) and the roll (should be right/left) to align the axes
   */
  // TODO: Fix with new ctre lib and move object scope to drivetrain subsystem
  public SimplifiedPigeon2(int deviceNumber, double pitchOffset, double rollOffset, boolean invertPitch, boolean invertRoll, boolean swapPitchAndRoll) {
    pigeon2 = new Pigeon2(deviceNumber);
    this.invertPitch = invertPitch ? -1 : 1;
    this.invertRoll = invertRoll ? -1 : 1;
    this.swapPitchAndRoll = swapPitchAndRoll;

    // Set up the mount pose and reset the yaw
    pigeon2.getConfigurator().apply(new MountPoseConfigs()
      .withMountPoseYaw(0)
      .withMountPosePitch(pitchOffset)
      .withMountPoseRoll(rollOffset)
    );

    // Configure status signals
    yawSignal = pigeon2.getYaw();
    yawSignal.setUpdateFrequency(50);

    pitchSignal = pigeon2.getPitch();
    pitchSignal.setUpdateFrequency(50);

    rollSignal = pigeon2.getRoll();
    rollSignal.setUpdateFrequency(50);

    // Remove all other automatic updates
    pigeon2.optimizeBusUtilization();
  }

  /**
   * Creates a SimplifiedPigeon2 with specified offsets. Assumes no inversions are necessary, e.g. where Pigeon is flat with the X-axis toward the front of the robot.
   *
   * @param deviceNumber CAN Device ID of the Pigeon
   * @param pitchOffset Additive pitch offset, best found by calibrating through PhoenixTuner
   * @param rollOffset Additive roll offset, best found by calibrating through PhoenixTuner
   */
  public SimplifiedPigeon2(int deviceNumber, double pitchOffset, double rollOffset) {
    this(deviceNumber, pitchOffset, rollOffset, false, false, false);
  }

  /** @return IMU yaw, counter-clockwise positive. If this IMU is being used for odometry, use the odometry heading instead of this method to avoid conflicting data due to odometry pose resets */
  public double getYaw() {
    return yawSignal.getValue();
  }

  /** @return IMU pitch, robot-front positive */
  public double getPitch() {
    if (swapPitchAndRoll) {
      return rollSignal.getValue() * invertPitch; // Invert the role if needed
    }
    return pitchSignal.getValue() * invertPitch;
  }

  /** @return IMU roll, robot-right positive */
  public double getRoll() {
    if (swapPitchAndRoll) {
      return pitchSignal.getValue() * invertRoll;
    }
    return rollSignal.getValue() * invertRoll;
  }

  /** @return IMU orientation as a {@link Rotation3d}. May cause mild performance issues */
  public Rotation3d getRotation3d() {
    return pigeon2.getRotation3d();
  }
}