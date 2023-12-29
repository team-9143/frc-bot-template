package frc.robot.util;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.logger.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants.DriveConsts;
import java.util.Map;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class Pathing {
  private static final String PATH_LOG_DIR = "/pathplanner/";

  private static final PathConstraints constraints = new PathConstraints(DriveConsts.kMaxWheelVelMetersPerSecond, DriveConsts.kMaxLinearAccelMetersPerSecondSquared);

  // Set up logging for basic path following
  static {
    PPSwerveControllerCommand.setLoggingCallbacks(
      traj -> Logger.recordOutput(PATH_LOG_DIR+"trajectory", traj.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new)), // Log trajectory on command initialization
      pose -> Logger.recordOutput(PATH_LOG_DIR+"referencePose", pose), // Log reference pose during command run
      speeds -> Logger.recordOutput(PATH_LOG_DIR+"setpointSpeeds", speeds), // Log setpoint velocities during command run
      null // Don't need to log error
    );
  }

  /** Utility method to retrieve a straight-line pose targeting command.
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
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   * @return the command
   */
  public static Command getFollowPathplannerCommand(String name) {
    var command = new PPSwerveControllerCommand(
      PathPlanner.loadPath(name, constraints), // Path to follow
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

  /**
   * Create a complete autonomous command group. This will reset the robot pose at the begininng of
   * the first path, follow paths, trigger events during path following, and run commands between
   * paths with stop events.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   * @param events String-Command map of named commands to run at events. Names may be reused for multiple events. Commands that run at stop events may require the drivetrain, but no others should.
   * @return the command
   */
  public static Command getFollowPathplannerWithEventsCommand(String name, Map<String, Command> events) {
    var builder = new SwerveAutoBuilder(
      Drivetrain.getInstance()::getPose, // Pose supplier
      Drivetrain.getInstance()::resetOdometry, // Pose consumer to set the odometry to the start of the path
      new PIDConstants(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDConstants(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
      events, // Commands to run at named events
      true, // Whether to mirror path to match alliance
      Drivetrain.getInstance() // Subsystem requirements
    );

    // Load and compute path
    return builder.fullAuto(PathPlanner.loadPathGroup(name, constraints));
  }
}