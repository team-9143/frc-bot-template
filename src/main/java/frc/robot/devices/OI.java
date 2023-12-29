package frc.robot.devices;

import frc.robot.Constants.DeviceConsts;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OI {
  public final static Controller DRIVER_CONTROLLER = new Controller(DeviceConsts.kDriverPort);
  public final static Controller OPERATOR_CONTROLLER = new Controller(DeviceConsts.kOperatorPort);

  // Remember to calibrate and update pitch and roll offset
  public final static SimplifiedPigeon2 IMU = new SimplifiedPigeon2(DeviceConsts.kPigeonID, 0, 0);

  public final static Limelight LIMELIGHT = new Limelight(NetworkTableInstance.getDefault().getTable("limelight"));
}