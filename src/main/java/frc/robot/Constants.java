package frc.robot;

import frc.robot.util.TunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import edu.wpi.first.math.controller.PIDController;

/** Global constants. Should not include functional code. */
public class Constants {
  /** Non-functional information for software testing and metadata. */
  public static class Config {
    /** {@code true} to stream log file data to NetworkTables (takes up bandwith and processing time, but useful for concurrent running and visualization) */
    public static final boolean NTStream = false;
    /** Directory for log file. Leave blank to store in local /logs/ directory. */
    public static final String DATA_LOG_DIR = "";
  }

  /** Ports and properties of non-motor devices. */
  public static class DeviceConsts {
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 5;

    // Pitch and roll offsets - REMEMBER TO UPDATE
    public static final double kPigeonPitchOffset = 0;
    public static final double kPigeonRollOffset = 0;
  }

  /** Physical parts of the robot, such as gearboxes or wheel diameters. */
  public static class PhysConsts {
    public static final int kNEOMaxVoltage = 12; // NEO V1.1 nominal voltage
    public static final int kNEOCurrentLimit = 40; // NEO V1.1 general current limit (40A-60A is advised)

    public static final double kSwerveDriveRPS = 5680d / 60d; // NEO V1.1 empirical free speed
    public static final double kSwerveDriveGearbox = 1d/5.355; // SDS L3 modules with 16T drive pinion
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI; // Billet wheels
  }

  /** Data relating to the entire drivetrain. */
  public static class DriveConsts {
    // Upper bound drivetrain constraints
    public static final double kMaxLinearVelMetersPerSecond = PhysConsts.kSwerveDriveRPS * PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters * 0.8; // 80% of theoretical max (motor RPS * gearbox * wheel circumfrence * 80%)
    public static final double kMaxTurnVelRadiansPerSecond = kMaxLinearVelMetersPerSecond / Constants.SwerveConsts.kSwerve_fl.location.getDistance(new Translation2d()); // ω = velocity / radius (assuming square swerve drivetrain)
    public static final double kMaxModuleAzimuthVoltage = 0.65 * PhysConsts.kNEOMaxVoltage; // Maximum azimuth motor voltage

    // Multipliers for all teleop driving
    public static final double kTeleopSpeedMult = 1;
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond; // Set maximum teleop turn speed to 1.5 rotations/s
  }

  /** Data for each individual swerve module. */
  public static class SwerveConsts {
    // Gains for module velocity error -> voltage
    public static final TunableNumber
      kDriveS = new TunableNumber("S", 0, "Module Drive"),
      kDriveP = new TunableNumber("P", 1.5e-2, "Module Drive");

    // Gains for module azimuth error (degrees) -> voltage
    public static final TunableNumber
      kAzimuthS = new TunableNumber("S", 0, "Module Azimuth"),
      kAzimuthV = new TunableNumber("V", 0, "Module Azimuth"),
      kAzimuthP = new TunableNumber("P", 0.0065, "Module Azimuth"),
      kAzimuthD = new TunableNumber("D", 0.00005, "Module Azimuth");

    public static final SwerveModuleConstants
      kSwerve_fl = new SwerveModuleConstants(
        "/module-front-left/", 11, 12, 13, -0.609619 * 360d,
        new Translation2d(0.14605, 0.24765),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAzimuthP.getAsDouble(), 0, kAzimuthD.getAsDouble())
      ),
      kSwerve_fr = new SwerveModuleConstants(
        "/module-front-right/", 21, 22, 23, -0.666504 * 360d,
        new Translation2d(0.14605, -0.24765),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAzimuthP.getAsDouble(), 0, kAzimuthD.getAsDouble())
      ),
      kSwerve_bl = new SwerveModuleConstants(
        "/module-back-left/", 31, 32, 33, -0.589355 * 360d,
        new Translation2d(-0.24765, 0.24765),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAzimuthP.getAsDouble(), 0, kAzimuthD.getAsDouble())
      ),
      kSwerve_br = new SwerveModuleConstants(
        "/module-back-right/", 41, 42, 43, -0.687988 * 360d,
        new Translation2d(-0.24765, -0.24765),
        new PIDController(kDriveP.getAsDouble(), 0, 0),
        new PIDController(kAzimuthP.getAsDouble(), 0, kAzimuthD.getAsDouble())
      );

    // Bind Tunables
    static {
      kDriveP.bind(val -> {
        kSwerve_fl.speed_controller.setP(val);
        kSwerve_fr.speed_controller.setP(val);
        kSwerve_bl.speed_controller.setP(val);
        kSwerve_br.speed_controller.setP(val);
      });
      kAzimuthP.bind(val -> {
        kSwerve_fl.azimuth_controller.setP(val);
        kSwerve_fr.azimuth_controller.setP(val);
        kSwerve_bl.azimuth_controller.setP(val);
        kSwerve_br.azimuth_controller.setP(val);
      });
      kAzimuthD.bind(val -> {
        kSwerve_fl.azimuth_controller.setD(val);
        kSwerve_fr.azimuth_controller.setD(val);
        kSwerve_bl.azimuth_controller.setD(val);
        kSwerve_br.azimuth_controller.setD(val);
      });
    }
  }

  public static class AutoConsts {
    // Upper bound drivetrain accelerations for path following and pose targeting
    public static final double kMaxLinearAccelMetersPerSecondSquared = DriveConsts.kMaxLinearVelMetersPerSecond  / 0.5; // Reaches max speed in 0.5 seconds
    public static final double kMaxTurnAccelRadiansPerSecondSquared = DriveConsts.kMaxTurnVelRadiansPerSecond / 0.5; // Reaches max speed in 0.5 seconds

    // Gains for drivetrain position error -> velocity
    public static final TunableNumber
      kTranslateP = new TunableNumber("P", 1, "Robot Translation"),
      kTranslateI = new TunableNumber("I", 0, "Robot Translation"),
      kTranslateD = new TunableNumber("D", 0, "Robot Translation");
    public static final TunableNumber
      kRotateP = new TunableNumber("P", 1, "Robot Rotation"),
      kRotateI = new TunableNumber("I", 0, "Robot Rotation"),
      kRotateD = new TunableNumber("D", 0, "Robot Rotation");

    // TODO: Create and register named commands here
    //static List<Pair<String, Command>> commands
  }
}