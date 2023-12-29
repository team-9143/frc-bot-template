package frc.robot.devices;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.Constants.DeviceConsts;

/** This class is meant to remove much of the functionality of the Pigeon2 so that data cannot be set by unauthorized code. If new data needs to be available, wrapper methods may be added easily to this class. */
public class SimplifiedIMU {
  // In proper orientation, Pigeon is flat with the X-axis toward the front of the robot
  /** Roll increases to robot-right, pitch to robot-front, and yaw counter-clockwise. */
  private final static Pigeon2 PIGEON2 = new Pigeon2(DeviceConsts.kPigeonID);
  static {
    // Remember to calibrate and update pitch and roll offsets
    PIGEON2.configMountPose(0, 0, 0);
    PIGEON2.setYaw(0);
  }

  /** @return IMU yaw, counter-clockwise positive */
  public static double getYaw() {
    return PIGEON2.getYaw();
  }

  /** @return IMU pitch, robot-front positive */
  public static double getPitch() {
    return PIGEON2.getPitch();
  }

  /** @return IMU roll, robot-right positive */
  public static double getRoll() {
    return PIGEON2.getRoll();
  }
}