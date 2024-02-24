package frc.robot.util;

import frc.robot.Constants.PhysConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.SwerveConsts;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.logger.LoggedSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.logger.LoggedCANcoder;

/** Controls a single swerve module. */
public class SwerveModule {
  private final LoggedSparkMax drive_motor;
  private final LoggedSparkMax azimuth_motor;
  private final LoggedCANcoder cancoder;

  private final PIDController speed_controller;
  private final PIDController azimuth_controller;

  protected SwerveModule(SwerveModuleConstants constants) {
    drive_motor = new LoggedSparkMax(constants.drive_ID, MotorType.kBrushless, constants.directory + "/drive/", PhysConsts.kNEOMaxVoltage, PhysConsts.kNEOCurrentLimit);
    azimuth_motor = new LoggedSparkMax(constants.azimuth_ID, MotorType.kBrushless, constants.directory + "/angle/", DriveConsts.kMaxModuleAzimuthVoltage, PhysConsts.kNEOCurrentLimit);
    azimuth_motor.setInverted(constants.azimuthInverted);
    cancoder = new LoggedCANcoder(constants.cancoder_ID, constants.directory, constants.cancoderOffset);
    speed_controller = constants.speed_controller;
    azimuth_controller = constants.azimuth_controller;

    // Set up drive encoder units
    drive_motor.encoder.setPositionConversionFactor(PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters); // UNIT: meters
    drive_motor.encoder.setVelocityConversionFactor(PhysConsts.kSwerveDriveGearbox * PhysConsts.kSwerveWheelCircumferenceMeters / 60); // UNIT: meters/s
    drive_motor.encoder.setMeasurementPeriod(20);
    drive_motor.encoder.setPosition(0);

    // Set up speed PID controller
    speed_controller.setSetpoint(0);

    // Set up rotational PID controller
    azimuth_controller.enableContinuousInput(-180, 180);
    azimuth_controller.setSetpoint(0);
  }

  /**
   * Calculate and set swerve module speed.
   *
   * @param speed module speed (UNIT: meters/s)
   * @param angle module angle (UNIT: ccw degrees)
   */
  protected void drive(double speed, double angle) {
    // Calculate and set azimuth motor speed
    azimuth_motor.setVoltage(
      SwerveConsts.kAzimuthS.getAsDouble() * Math.signum(angle - getAngle()) // Simple static feedforward
      + azimuth_controller.calculate(getAngle(), angle) // Azimuth feedback controller
    );

    // Calculate and set drive motor speed
    drive_motor.setVoltage(
      Math.max(-PhysConsts.kNEOMaxVoltage, Math.min(PhysConsts.kNEOMaxVoltage, // Clamp to nominal voltage
        SwerveConsts.kDriveS.getAsDouble() * Math.signum(speed) // Simple static feedforward
        + (PhysConsts.kNEOMaxVoltage * speed/DriveConsts.kMaxLinearVelMetersPerSecond) // Simple velocity feedforward
        + speed_controller.calculate(getVelocity(), speed) // Velocity adjustment feedback controller
      )) * Math.abs(Math.cos(getAngleError() * Math.PI/180)) // Scale velocity down if not at proper angle
    );
  }

  /**
   * Drive the swerve module based on a desired angle and speed.
   *
   * @param desired desired state
   */
  protected void desiredStateDrive(SwerveModuleState desired) {
    desired = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(getAngle())); // Optimize so that maximum required turn is 1/4 circle
    drive(
      desired.speedMetersPerSecond,
      desired.angle.getDegrees()
    );
  }

  /** @return the angle of the module (UNIT: ccw degrees) */
  public double getAngle() {
    return cancoder.getOffsetPosition();
  }

  /** @return the velocity of the module (UNIT: meters/s) */
  public double getVelocity() {
    return drive_motor.encoder.getVelocity();
  }

  /** @return the current error in the angle of the module (UNIT: ccw degrees) */
  public double getAngleError() {
    return azimuth_controller.getPositionError();
  }

  /** @return the distance traveled by the module (UNIT: meters) */
  public double getDistance() {
    return drive_motor.encoder.getPosition();
  }

  public void stopMotor() {
    drive_motor.stopMotor();
    azimuth_motor.stopMotor();

    azimuth_controller.reset();
    speed_controller.reset();
  }

  /** Basic constants for the construction of a {@link SwerveModule}. */
  public static class SwerveModuleConstants {
    public final String directory;

    public final byte drive_ID;
    public final byte azimuth_ID;
    public final byte cancoder_ID;
    public final boolean azimuthInverted;
    public final double cancoderOffset;
    public final Translation2d location;

    public final PIDController speed_controller;
    public final PIDController azimuth_controller;

    /**
     * @param directory sub-directory for logging (with trailing slash)
     * @param drive_ID driving motor ID (Spark Max with brushless motor)
     * @param azimuth_ID azimuth motor ID (Spark Max with brushless motor)
     * @param cancoder_ID cancoder ID
     * @param azimuthInverted if the azimuth motor should be inverted (use on mk4i)
     * @param cancoderOffset additive cancoder offset (UNIT: ccw degrees)
     * @param location location of the wheel relative to the center of rotation of the robot (forward, left) (UNIT: meters)
     * @param speed_controller PID controller to calculate drive motor speed from velocity error
     * @param azimuth_controller PID controller to calculate azimuth motor speed from degree error
     */
    public SwerveModuleConstants(String directory, int drive_ID, int azimuth_ID, int cancoder_ID, boolean azimuthInverted, double cancoderOffset, Translation2d location, PIDController speed_controller, PIDController azimuth_controller) {
      this.directory = directory;
      this.drive_ID = (byte) drive_ID;
      this.azimuth_ID = (byte) azimuth_ID;
      this.cancoder_ID = (byte) cancoder_ID;
      this.azimuthInverted = azimuthInverted;
      this.cancoderOffset = cancoderOffset;
      this.location = location;
      this.speed_controller = speed_controller;
      this.azimuth_controller = azimuth_controller;
    }
  }
}