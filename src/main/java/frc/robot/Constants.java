package frc.robot;

import frc.robot.util.TunableNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;

/** Global constants. Should not include functional code. */
public class Constants {
  /** Non-functional information for software testing and metadata. */
  public static class Config {
    /** {@code true} to stream log file data to NetworkTables (takes up bandwith and processing time, but useful for concurrent running and visualization) */
    public static final boolean NTStream = false;
    /** Directory for log file. Leave blank to store in project directory. */
    public static final String DATA_LOG_DIR = "";
  }

  /** Ports and properties of non-motor devices. */
  public static class DeviceConsts {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  /** Physical parts of the robot, such as gearboxes or wheel diameters. */
  public static class PhysConsts {
    public static final double kSwerveWheelGearbox = 1/5.14; // SDS L4 modules
    public static final double kSwerveWheelCircumferenceMeters = 0.09779 * Math.PI; // Colson wheels
  }

  /** Data relating to the entire drivetrain. */
  public static class DriveConsts {
    // Multipliers for all teleop driving
    public static final double kTeleopSpeedMult = 1;
    public static final double kTeleopTurnMult = 0.7;

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

  /** Data for each individual swerve module. */
  public static class SwerveConsts {
    public static final TunableNumber
      kDriveP = new TunableNumber("P", 1.5e-2, "Module Drive");
    public static final TunableNumber
      kAngleP = new TunableNumber("P", 0.0065, "Module Angle"),
      kAngleD = new TunableNumber("D", 0.00005, "Module Angle");

    public static final SwerveModuleConstants
      kSwerve_fl = new SwerveModuleConstants(
        "module_FL", 41, 42, 43, 31.465,
        new Translation2d(0.22225, 0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_fr = new SwerveModuleConstants(
        "module_FR", 11, 12, 13, 28.037,
        new Translation2d(0.22225, -0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_bl = new SwerveModuleConstants(
        "module_BL", 31, 32, 33, 86.748,
        new Translation2d(-0.22225, 0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      ),
      kSwerve_br = new SwerveModuleConstants(
        "module_BR", 21, 22, 23, -96.943,
        new Translation2d(-0.22225, -0.22225),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAngleP.getAsDouble(), 0, kAngleD.getAsDouble())
      );

    // Bind Tunables
    static {
      kDriveP.bindTo(val -> {
        kSwerve_fl.speed_controller.setP(val);
        kSwerve_fr.speed_controller.setP(val);
        kSwerve_bl.speed_controller.setP(val);
        kSwerve_br.speed_controller.setP(val);
      });
      kAngleP.bindTo(val -> {
        kSwerve_fl.angle_controller.setP(val);
        kSwerve_fr.angle_controller.setP(val);
        kSwerve_bl.angle_controller.setP(val);
        kSwerve_br.angle_controller.setP(val);
      });
      kAngleD.bindTo(val -> {
        kSwerve_fl.angle_controller.setD(val);
        kSwerve_fr.angle_controller.setD(val);
        kSwerve_bl.angle_controller.setD(val);
        kSwerve_br.angle_controller.setD(val);
      });
    }
  }
}