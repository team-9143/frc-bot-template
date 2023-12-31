package frc.robot.util;

import edu.wpi.first.wpilibj.MotorSafety;

import frc.robot.Constants.DriveConsts;
import frc.robot.devices.OI;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

/** Controls a set of four {@link SwerveModule SwerveModules}. Protected by {@link MotorSafety}, and speeds must be set every iteration. */
public class SwerveDrive extends MotorSafety {
  public final SwerveModule[] modules;
  private SwerveModuleState[] desiredStates = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  public final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;

  public SwerveDrive(SwerveModule.SwerveModuleConstants consts_fl, SwerveModule.SwerveModuleConstants consts_fr, SwerveModule.SwerveModuleConstants consts_bl, SwerveModule.SwerveModuleConstants consts_br) {
    // Turn on motor safety watchdog
    setSafetyEnabled(true);

    // Initialize swerve modules
    modules = new SwerveModule[] {
      new SwerveModule(consts_fl),
      new SwerveModule(consts_fr),
      new SwerveModule(consts_bl),
      new SwerveModule(consts_br)
    };

    // Initialize kinematics
    kinematics = new SwerveDriveKinematics(
      consts_fl.location,
      consts_fr.location,
      consts_bl.location,
      consts_br.location
    );

    // Initialize odometry - If using vision adjustments, add standard deviation matrices
    odometry = new SwerveDrivePoseEstimator(
      kinematics,
      Rotation2d.fromDegrees(OI.IMU.getYaw()),
      new SwerveModulePosition[] {
        new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
        new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
        new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
        new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
      },
      new Pose2d() // If using field-relative paths, use field-relative pose here
    );
  }

  /** Updates odometry and pushes desired control to modules. Should be called every robot loop. */
  public void update() {
    // Update odometry state estimation
    odometry.update(Rotation2d.fromDegrees(OI.IMU.getYaw()), new SwerveModulePosition[] {
      new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
    });

    // Normalize and push angles and speeds to modules
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConsts.kMaxWheelVelMetersPerSecond);
    modules[0].desiredStateDrive(desiredStates[0]);
    modules[1].desiredStateDrive(desiredStates[1]);
    modules[2].desiredStateDrive(desiredStates[2]);
    modules[3].desiredStateDrive(desiredStates[3]);
  }

  /** Sets desired module states, without optimizing. */
  public void setDesiredStates(SwerveModuleState state_fl, SwerveModuleState state_fr, SwerveModuleState state_bl, SwerveModuleState state_br) {
    desiredStates[0] = state_fl;
    desiredStates[1] = state_fr;
    desiredStates[2] = state_bl;
    desiredStates[3] = state_br;

    feed();
  }

  /**
   * Sets desired module states from field relative velocities.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public void setDesiredVelocityFieldRelative(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forward, left, ccw, getPose().getRotation()));
    setDesiredStates(states[0], states[1], states[2], states[3]);
  }

  /**
   * Sets desired module states from robot relative velocities.
   *
   * @param forward forward speed (UNIT: meters/s)
   * @param left left speed (UNIT: meters/s)
   * @param ccw counter-clockwise speed (UNIT: ccw radians/s)
   */
  public void setDesiredVelocity(double forward, double left, double ccw) {
    var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, left, ccw));
    setDesiredStates(states[0], states[1], states[2], states[3]);
  }

  /**
   * Reset the odometry to a given position.
   *
   * @param positionMetersCCW robot position (UNIT: meters, ccw native angle)
   */
  public void resetOdometry(Pose2d positionMetersCCW) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(OI.IMU.getYaw()),
      new SwerveModulePosition[] {
        new SwerveModulePosition(modules[0].getDistance(), Rotation2d.fromDegrees(modules[0].getAngle())),
        new SwerveModulePosition(modules[1].getDistance(), Rotation2d.fromDegrees(modules[1].getAngle())),
        new SwerveModulePosition(modules[2].getDistance(), Rotation2d.fromDegrees(modules[2].getAngle())),
        new SwerveModulePosition(modules[3].getDistance(), Rotation2d.fromDegrees(modules[3].getAngle()))
      },
      positionMetersCCW
    );
  }

  /** @return the robot's estimated location */
  public Pose2d getPose() {return odometry.getEstimatedPosition();}

  /** @return the drivetrain's desired velocities */
  public ChassisSpeeds getDesiredSpeeds() {return kinematics.toChassisSpeeds(desiredStates);}

  /** @return the drivetrain's actual velocities, as measured by encoders */
  public ChassisSpeeds getMeasuredSpeeds() {return kinematics.toChassisSpeeds(getMeasuredStates());}

  /** @return individual desired module states */
  public SwerveModuleState[] getDesiredStates() {return desiredStates;}

  /** @return individual measured module states */
  public SwerveModuleState[] getMeasuredStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(modules[0].getVelocity(), Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModuleState(modules[1].getVelocity(), Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModuleState(modules[2].getVelocity(), Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModuleState(modules[3].getVelocity(), Rotation2d.fromDegrees(modules[3].getAngle()))
    };
  }

  /** Stop the modules and reset the desired states. */
  @Override
  public void stopMotor() {
    for (SwerveModule module : modules) {module.stopMotor();}

    setDesiredStates(
      new SwerveModuleState(0, Rotation2d.fromDegrees(modules[0].getAngle())),
      new SwerveModuleState(0, Rotation2d.fromDegrees(modules[1].getAngle())),
      new SwerveModuleState(0, Rotation2d.fromDegrees(modules[2].getAngle())),
      new SwerveModuleState(0, Rotation2d.fromDegrees(modules[3].getAngle()))
    );
  }

  @Override
  public String getDescription() {
    return "Swerve Drive";
  }
}