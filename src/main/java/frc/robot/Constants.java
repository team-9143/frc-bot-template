package frc.robot;

import frc.robot.util.TunableNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
    public static final double kMaxWheelVelMetersPerSecond = 6 * 0.8; // 80% of theoretical max 6 m/s
    public static final double kMaxTurnVelRadiansPerSecond = 9.5; // 1.5 rotations/sec
    public static final double kMaxModuleRotateSpeedPercentage = 0.65; // Maximum rotational motor speed

    // Upper bound drivetrain accelerations for path following and pose targeting
    public static final double kMaxLinearAccelMetersPerSecondSquared = kMaxWheelVelMetersPerSecond * 2; // UNIT: meters/s/s
    public static final double kMaxTurnAccelRadiansPerSecondSquared = kMaxTurnVelRadiansPerSecond * 2; // UNIT: radians/s/s

    // Gains for drivetrain position error -> desired velocity
    public static final TunableNumber
      kTranslateP = new TunableNumber("P", 1, "Robot Translation"),
      kTranslateI = new TunableNumber("I", 0, "Robot Translation"),
      kTranslateD = new TunableNumber("D", 0, "Robot Translation");
    public static final TunableNumber
      kRotateP = new TunableNumber("P", 1, "Robot Rotation"),
      kRotateI = new TunableNumber("I", 0, "Robot Rotation"),
      kRotateD = new TunableNumber("D", 0, "Robot Rotation");

    // Drivetrain location control tolerance
    public static final Pose2d kPosTolerance = new Pose2d(
      new Translation2d(0.0127, 0.0127), // UNIT: meters
      Rotation2d.fromDegrees(0.75)
    );
  }
}