package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConstants;
import frc.robot.util.TunableNumber;

/** Global constants. Should not include functional code. */
public class Constants {
  /** Information for testing and robot configuration that must be updated consistenly. */
  public static class Config {
    /**
     * Set true to stream log file data to NetworkTables (takes up bandwith and processing time, but
     * useful for concurrent running and visualization)
     */
    public static final boolean NTStream = false;

    // TODO(user): Update pigeon offsets
    // Pigeon mount offsets - REMEMBER TO UPDATE, or configure through PhoenixTuner
    public static final MountPoseConfigs kPigeonMountPose =
        new MountPoseConfigs().withMountPoseYaw(0).withMountPosePitch(0).withMountPoseRoll(0);
  }

  /** Ports and properties of non-motor devices. */
  public static class DeviceConsts {
    // TODO(user): Update device ID's
    public static final byte kDriverPort = 0;
    public static final byte kOperatorPort = 1;
    public static final byte kPigeonID = 2;
  }

  // TODO(user): Fix all physical robot constraints as design dictates
  /** Physical parts of the robot, such as gearboxes or wheel diameters. */
  public static class PhysConsts {
    /** General nominal motor voltage */
    public static final int kMaxVoltage = 12;

    /** General current limit (40A-60A is advised for NEO V1.1's) */
    public static final int kDefaultCurrentLimit = 40;

    /** NEO V1.1 empirical free speed */
    public static final double kSwerveDriveMaxRPS = 5680d / 60d;

    /** Ratio of drive wheels to motors for SDS L3 modules with 16T drive pinions */
    public static final double kSwerveDriveMechToSens = 1d / 5.355;

    /** Billet wheels */
    public static final double kSwerveWheelCircumferenceMeters = 0.099 * Math.PI;
  }

  // TODO(user): Check all drivetrain measurements and limits and ensure accuracy
  /** Data relating to the entire drivetrain. */
  public static class DriveConsts {
    // Upper bound drivetrain constraints
    /** 80% of theoretical max */
    public static final double kMaxLinearVelMetersPerSecond =
        PhysConsts.kSwerveDriveMaxRPS
            * PhysConsts.kSwerveDriveMechToSens
            * PhysConsts.kSwerveWheelCircumferenceMeters
            * 0.8;

    // ω = velocity / radius
    public static final double kMaxTurnVelRadiansPerSecond =
        kMaxLinearVelMetersPerSecond / Constants.SwerveConsts.kDriveBaseRadius;

    // To avoid brownouts and overpowering
    public static final double kModuleAzimuthMaxVoltage = 0.65 * PhysConsts.kMaxVoltage;
    public static final double kModuleDriveMaxVoltage = 0.95 * PhysConsts.kMaxVoltage;
    public static final int kModuleAzimuthCurrentLimit = 30;

    // Multipliers for all teleop driving
    public static final double kTeleopSpeedMult = 1;
    // Set maximum teleop turn speed to 1.5 rotations/s
    public static final double kTeleopTurnMult = 9.5 / kMaxTurnVelRadiansPerSecond;

    /** Update rate for drivetrain, default 20 ms. 8-64 ms is best for NEO hall sensor. */
    public static final int kPeriodMs = 10;
  }

  /** Data for each individual swerve module. */
  public static class SwerveConsts {
    // TODO(user): Tune swerve propulsion gains
    // Gains for module velocity error -> voltage
    public static final TunableNumber kDriveS = new TunableNumber("S", 0.1, "Module Drive");
    public static final TunableNumber kDriveP = new TunableNumber("P", 2, "Module Drive");

    // Whether azimuth motor is inverted, use for mk4i's
    public static final boolean kAzimuthInverted = true;

    // TODO(user): Update swerve module locations, ID's, cancoder offsets, and gains
    public static final SwerveModuleConstants
        kSwerve_fl =
            new SwerveModuleConstants(
                "SwerveFL",
                // Azimuth gains (kS, kP, kD)
                0.1,
                0.095,
                0.0006,
                // CANcoder offset
                -0.100342,
                11,
                12,
                13,
                new Translation2d(0.14605, 0.24765)),
        kSwerve_fr =
            new SwerveModuleConstants(
                "SwerveFR",
                // Azimuth gains (kS, kP, kD)
                0.092,
                0.1,
                0.00065,
                // CANcoder offset
                -0.679443,
                21,
                22,
                23,
                new Translation2d(0.14605, -0.24765)),
        kSwerve_bl =
            new SwerveModuleConstants(
                "SwerveBL",
                // Azimuth gains (kS, kP, kD)
                0.08,
                0.105,
                0.0004,
                // CANcoder offset
                -0.088867,
                31,
                32,
                33,
                new Translation2d(-0.24765, 0.24765)),
        kSwerve_br =
            new SwerveModuleConstants(
                "SwerveBR",
                // Azimuth gains (kS, kP, kD)
                0.092,
                0.09,
                0.00065,
                // CANcoder offset
                -0.679443,
                41,
                42,
                43,
                new Translation2d(-0.24765, -0.24765));

    /** Drive base radius for angular velocity calcs (use swerve module farthest from COR) */
    public static final double kDriveBaseRadius =
        kSwerve_bl.location.getDistance(new Translation2d());
  }

  public static class AutoConsts {
    // TODO(dev/user): Ensure that drivetrain acceleration limits are strong
    // Upper bound drivetrain accelerations for path following and pose targeting
    public static final double kMaxLinearAccelMetersPerSecondSquared =
        DriveConsts.kMaxLinearVelMetersPerSecond;
    public static final double kMaxTurnAccelRadiansPerSecondSquared =
        DriveConsts.kMaxTurnVelRadiansPerSecond;

    // TODO(user): Tune pathfinding PID gains (currently at default gains)
    // Gains for drivetrain position error -> velocity
    public static final TunableNumber kTranslateP = new TunableNumber("P", 5, "Robot Translation");
    public static final TunableNumber kTranslateD = new TunableNumber("D", 0, "Robot Translation");
  }
}
