package frc.robot;

/** Global constants. Should not include functional code. */
public class Constants {
  public static class DeviceConsts {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  public static class PhysConsts {
    public static final double kSwerveWheelGearbox = 1/5.14; // SDS L4 modules
    public static final double kSwerveWheelCircumferenceMeters = 0.09779 * Math.PI; // Colson wheels
  }

  public static class DriveConsts {
    // Upper bound drivetrain constraints
    public static final double kMaxLinearVelMetersPerSecond = 6 * 0.8; // 80% of theoretical max
    public static final double kMaxTurnVelRadiansPerSecond = 9.5; // 1.5 rotations/sec
    public static final double kMaxModuleRotateSpeedPercentage = 0.65; // Maximum rotational motor speed
  }
}