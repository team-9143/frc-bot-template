package frc.robot.util;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.logger.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.DriveConsts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class Pathing {
  private static final String PATH_LOG_DIR = "/paths/";

  // Set up logging for basic path following
  static {
    PPSwerveControllerCommand.setLoggingCallbacks(
      traj -> Logger.recordOutput(PATH_LOG_DIR+"basic-follow/trajectory", traj.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new)), // Log trajectory on command initialization
      pose -> Logger.recordOutput(PATH_LOG_DIR+"basic-follow/referencePose", pose), // Log reference pose during command run
      speeds -> Logger.recordOutput(PATH_LOG_DIR+"basic-follow/setpointSpeeds", speeds), // Log setpoint velocities during command run
      (t, r) -> {} // Don't need to log error
    );
  }

  /** Utility method to retrieve an easy direct move command. Would not recommend using this over any pathing software.
   *
   * @param desiredPoseMetersCCW robot pose relative to the same origin as the odometry (UNIT: meters, ccw native angle)
   * @return the command
   */
  public static Command getDirectMoveCommand(Pose2d desiredPoseMetersCCW) {
    return Drivetrain.getInstance().run(() -> Drivetrain.getInstance().driveTargetPose(desiredPoseMetersCCW));
  }

  /**
   * Follow a pathplanner path, constrained by maximum robot velocity and maximum controlled acceleration. Logs everything on command run through Logger.
   *
   * @param name name of path file name under [deploy/pathplanner/], omitting ".path"
   * @return the command
   */
  public static Command getFollowPathplannerCommand(String name) {
    var command = new PPSwerveControllerCommand(
      PathPlanner.loadPath(name, DriveConsts.kMaxWheelVelMetersPerSecond, DriveConsts.kMaxLinearAccelMetersPerSecondSquared), // Path to follow
      Drivetrain.getInstance()::getPose, // Pose supplier
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
      true, // Whether to mirror path to match alliance
      Drivetrain.getInstance() // Subsystem requirements
    );
    return command;
  }
}