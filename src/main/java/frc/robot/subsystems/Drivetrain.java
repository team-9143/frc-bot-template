package frc.robot.subsystems;

import frc.robot.util.SafeSubsystem;

import frc.robot.util.SwerveDrive;

import frc.robot.devices.OI;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  /**
   * Drive to a position, relative to the odometry. Must be continuously called.
   *
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw counter-clockwise angle (UNIT: ccw degrees)
   * @param desiredLinearVelocityMetersPerSecond desired linear velocity for feedforward
   */
  public void driveToLocation(double forward, double left, double ccw, double desiredLinearVelocityMetersPerSecond) {
    m_swerve.setDesiredPose(
      new Pose2d(forward, left, Rotation2d.fromDegrees(ccw)),
      desiredLinearVelocityMetersPerSecond
    );
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
   * @param forward forward distance (UNIT: meters)
   * @param left left distance (UNIT: meters)
   * @param ccw counter-clockwise angle (UNIT: ccw degrees)
   */
  public void resetOdometry(double forward, double left, double ccw) {
    m_swerve.resetOdometry(new Pose2d(forward, left, Rotation2d.fromDegrees(ccw)));
  }

  /** @return the robot's estimated location */
  public Pose2d getPose() {return m_swerve.getPose();}

  /** @return {@code true} if trajectory following and near desired location */
  public boolean atReference() {return m_swerve.atReference();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return m_swerve.getDesiredSpeeds();}

  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getMeasuredSpeeds() {return m_swerve.getMeasuredSpeeds();}

  /** @return individual desired module states */
  public SwerveModuleState[] getDesiredStates() {return m_swerve.getDesiredStates();}

  /** @return individual measured module states */
  public SwerveModuleState[] getMeasuredStates() {return m_swerve.getMeasuredStates();}

  @Override
  public void stop() {
    m_swerve.stopMotor();
  }
}