package frc.robot.subsystems;

import frc.robot.logger.Logger;
import frc.robot.util.SwerveDrive;

import frc.robot.devices.OI;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Controls the robot drivetrain. */
public class Drivetrain extends SafeSubsystem {
  private static Drivetrain m_instance;

  /** @return the singleton instance */
  public static synchronized Drivetrain getInstance() {
    if (m_instance == null) {
      m_instance = new Drivetrain();
      m_instance.stop();
    }
    return m_instance;
  }

  public static final SwerveModuleState[] xStanceStates = new SwerveModuleState[] {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };

  private static final SwerveDrive m_swerve = new SwerveDrive(
    SwerveConsts.kSwerve_fl,
    SwerveConsts.kSwerve_fr,
    SwerveConsts.kSwerve_bl,
    SwerveConsts.kSwerve_br
  );

  private Drivetrain() {
    // Default drive command
    setDefaultCommand(run(() -> {
      double forward = -OI.DRIVER_CONTROLLER.getLeftY();
      double left = -OI.DRIVER_CONTROLLER.getLeftX();
      double ccw = -OI.DRIVER_CONTROLLER.getRightX(); // Right joystick horizontal for rotation

      // Field relative control, exponentially scaling inputs to increase sensitivity
      m_swerve.setDesiredVelocityFieldRelative(
        Math.copySign(forward*forward, forward) * DriveConsts.kMaxWheelVelMetersPerSecond * DriveConsts.kTeleopSpeedMult,
        Math.copySign(left*left, left) * DriveConsts.kMaxWheelVelMetersPerSecond * DriveConsts.kTeleopSpeedMult,
        Math.copySign(ccw*ccw*ccw, ccw) * DriveConsts.kMaxTurnVelRadiansPerSecond * DriveConsts.kTeleopTurnMult // Extra sensitivity for fine rotation control
      );
    }));
  }


  @Override
  public void periodic() {
    // Update swerve speeds and odometry
    m_swerve.update();
  }

  /**
   * Drive with field relative velocities. Must be continuously called.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public void driveFieldRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocityFieldRelative(forward, left, ccw);
  }

  /**
   * Drive with robot relative velocities. Must be continuously called.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public void driveRobotRelativeVelocity(double forward, double left, double ccw) {
    m_swerve.setDesiredVelocity(forward, left, ccw);
  }

  /** Set the drivetrain to x-stance for traction. Must be continuously called. */
  public void toXStance() {
    m_swerve.setDesiredStates(
      xStanceStates[0],
      xStanceStates[1],
      xStanceStates[2],
      xStanceStates[3]
    );
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param positionMetersCCW robot position (UNIT: meters, ccw native angle)
   */
  public void resetOdometry(Pose2d positionMetersCCW) {
    m_swerve.resetOdometry(positionMetersCCW);
  }

  /** @return the robot's estimated location */
  public Pose2d getPose() {return m_swerve.getPose();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return m_swerve.getDesiredSpeeds();}

  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getMeasuredSpeeds() {return m_swerve.getMeasuredSpeeds();}

  /** @return individual desired module states */
  public SwerveModuleState[] getDesiredStates() {return m_swerve.getDesiredStates();}

  /** @return individual measured module states */
  public SwerveModuleState[] getMeasuredStates() {return m_swerve.getMeasuredStates();}

  @Override
  public void log() {
    Logger.recordOutput(getDirectory()+"odometry", getPose());

    Logger.recordOutput(getDirectory()+"measuredStates", getMeasuredStates());
    Logger.recordOutput(getDirectory()+"desiredStates", getDesiredStates());

    Logger.recordOutput(getDirectory()+"measuredSpeeds", getMeasuredSpeeds());
    Logger.recordOutput(getDirectory()+"desiredSpeeds", getDesiredSpeeds());

    Logger.recordOutput(getDirectory()+"3dPosition",
      new Pose3d(getPose().getX(), getPose().getY(), 0, // Height always set to 0
      new Rotation3d(Math.toRadians(OI.IMU.getRoll()), Math.toRadians(OI.IMU.getPitch()), getPose().getRotation().getRadians())));
  }

  @Override
  public void stop() {
    m_swerve.stopMotor();
  }
}