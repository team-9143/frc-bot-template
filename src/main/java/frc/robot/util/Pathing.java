package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConsts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class Pathing {
  /** Utility method to retrieve an easy direct move command. Would not recommend using this over any pathing software.
   *
   * @param desiredPoseMetersCCW robot pose relative to the same origin as the odometry (UNIT: meters, ccw native angle)
   */
  public static Command getDirectMoveCommand(Pose2d desiredPoseMetersCCW) {
    return Drivetrain.getInstance().run(() -> Drivetrain.getInstance().driveToLocation(desiredPoseMetersCCW, DriveConsts.kMaxWheelVelMetersPerSecond));
  }

  /**
   * Get a Choreo trajectory, and send it to the Logger.
   *
   * @param name corresponding trajectory file name under [deploy/choreo/], omitting ".traj"
   */
  public static ChoreoTrajectory getChoreoTrajectory(String name) {
    return Choreo.getTrajectory(name);
  }

  /**
   * Create a follow command for a Choreo trajectory.
   *
   * @param traj corresponding trajectory
   * @param matchAlliance {@code true} to mirror the path if on the red alliance
   */
  public static Command getChoreoTrajectoryCommand(ChoreoTrajectory traj, boolean matchAlliance) {
    return Choreo.choreoSwerveCommand(
      traj,
      Drivetrain.getInstance()::getPose, // Pose supplier, needs to be field-relative
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      speeds -> Drivetrain.getInstance().driveRobotRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Robot-relative velocities consumer
      matchAlliance, // Whether to mirror path to match alliance
      Drivetrain.getInstance() // Subsystem requirements
    );
  }
}