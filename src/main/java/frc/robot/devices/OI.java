package frc.robot.devices;

import frc.robot.Constants.DeviceConstants;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.sensors.Pigeon2;

public class OI {
  public final static Controller DRIVER_CONTROLLER = new Controller(DeviceConstants.kDriverPort);
  public final static Controller OPERATOR_CONTROLLER = new Controller(DeviceConstants.kOperatorPort);

  // In proper orientation, Pigeon is flat and facing so that X-axis is forward
  /** Roll increases to the right, pitch to the front, and yaw counter-clockwise. */
  public final static Pigeon2 PIGEON2 = new Pigeon2(DeviceConstants.kPigeonID);
  static {
    // Remember to calibrate and update pitch and roll offsets
    PIGEON2.configMountPose(0, 0, 0);
    PIGEON2.setYaw(0);
  }

  public final static Limelight LIMELIGHT = new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
}