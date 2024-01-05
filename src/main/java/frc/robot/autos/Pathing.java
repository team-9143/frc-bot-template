package frc.robot.autos;

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
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import java.util.List;

/** Utility class for loading, generating, following, and logging paths through PathPlanner. PID controllers are initialized at runtime so that TunableNumbers can take effect. */
public class Pathing {
  private static final String PATH_LOG_DIR = "/pathplanner/";

  /** Default constraints for accurately generating and following paths */
  private static final PathConstraints default_constraints = new PathConstraints(DriveConsts.kMaxWheelVelMetersPerSecond, DriveConsts.kMaxLinearAccelMetersPerSecondSquared);

  // Set up logging for basic path following
  static {
    PPSwerveControllerCommand.setLoggingCallbacks(
      traj -> Logger.recordOutput(PATH_LOG_DIR+"trajectory", traj.getStates().stream().map(state -> state.poseMeters).toArray(Pose2d[]::new)), // Log trajectory on command initialization
      pose -> Logger.recordOutput(PATH_LOG_DIR+"referencePose", pose), // Log reference pose during command run
      speeds -> Logger.recordOutput(PATH_LOG_DIR+"setpointSpeeds", speeds), // Log setpoint velocities during command run
      null // Don't need to log error
    );
  }

  /******
   *
   * Path loading and generation
   *
   ******/

  /**
   * Load a pathplanner path, constrained by given maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   * @param maxVelMetersPerSecond max velocity along the path. Paths will not exceed the maximum velocity of the robot.
   * @param maxAccelMetersPerSecondPerSecond max acceleration along the path. Use {@code Double.POSITIVE_INFINITY} for immediate starts and stops.
   */
  public static PathPlannerTrajectory loadPath(String name, double maxVelMetersPerSecond, double maxAccelMetersPerSecondPerSecond) {
    return PathPlanner.loadPath(name, Math.min(maxVelMetersPerSecond, default_constraints.maxVelocity), maxAccelMetersPerSecondPerSecond);
  }

  /**
   * Load a pathplanner path, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   */
  public static PathPlannerTrajectory loadPath(String name) {
    return loadPath(name, default_constraints.maxVelocity, default_constraints.maxAcceleration);
  }

  /**
   * Load a pathplanner path, separated into a list by stop events, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   * @param maxVelMetersPerSecond max velocity along the path. Paths will not exceed the maximum velocity of the robot.
   * @param maxAccelMetersPerSecondPerSecond max acceleration along the path. Use {@code Double.POSITIVE_INFINITY} for immediate starts and stops.
   */
  public static List<PathPlannerTrajectory> loadPathGroup(String name, double maxVelMetersPerSecond, double maxAccelMetersPerSecondPerSecond) {
    return PathPlanner.loadPathGroup(name, Math.min(maxVelMetersPerSecond, default_constraints.maxVelocity), maxAccelMetersPerSecondPerSecond);
  }

  /**
   * Load a pathplanner path, separated into a list by stop events, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   */
  public static List<PathPlannerTrajectory> loadPathGroup(String name) {
    return loadPathGroup(name, default_constraints.maxVelocity, default_constraints.maxAcceleration);
  }

  /**
   * Create a pathplanner path, constrained by given maximum velocity and acceleration.
   *
   * @param startPoseMetersCCW starting position for the command
   * @param endPoseMetersCCW ending position for the command
   * @param maxVelMetersPerSecond max velocity along the path. Paths will not exceed the maximum velocity of the robot.
   * @param maxAccelMetersPerSecondPerSecond max acceleration along the path. Use {@code Double.POSITIVE_INFINITY} for immediate starts and stops.
   */
  public static PathPlannerTrajectory generateDirectPath(Pose2d startPoseMetersCCW, Pose2d endPoseMetersCCW, double maxVelMetersPerSecond, double maxAccelMetersPerSecondPerSecond) {
    // Find the angle between the poses for heading calculation
    Pose2d relativePose = endPoseMetersCCW.relativeTo(startPoseMetersCCW);
    Rotation2d relativeAngle = new Rotation2d(relativePose.getX(), relativePose.getY());

    // Create the path points from the given poses and the calculated heading
    PathPoint first = new PathPoint(startPoseMetersCCW.getTranslation(), relativeAngle, startPoseMetersCCW.getRotation());
    PathPoint last = new PathPoint(endPoseMetersCCW.getTranslation(), relativeAngle, endPoseMetersCCW.getRotation());

    // Generate the path with the constraints provided (velocity cannot exceed maximum robot velocity)
    return PathPlanner.generatePath(new PathConstraints(Math.min(maxVelMetersPerSecond, default_constraints.maxVelocity), maxAccelMetersPerSecondPerSecond), first, last);
  }

  /**
   * Create a pathplanner path, constrained by the default maximum velocity and acceleration.
   *
   * @param startPoseMetersCCW starting position for the command
   * @param endPoseMetersCCW ending position for the command
   */
  public static PathPlannerTrajectory generateDirectPath(Pose2d startPoseMetersCCW, Pose2d endPoseMetersCCW) {
    return generateDirectPath(startPoseMetersCCW, endPoseMetersCCW, default_constraints.maxVelocity, default_constraints.maxAcceleration);
  }

  /******
   *
   * Path following
   *
   ******/

  /**
   * Create a command to follow a pathplanner path. Logs everything through the Logger during the command run.
   *
   * @param path the pathplanner path to follow
   */
  public static Command getFollowPathplannerCommand(PathPlannerTrajectory path) {
    return new PPSwerveControllerCommand(
      path, // Path to follow
      Drivetrain.getInstance()::getPose, // Pose supplier
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDController(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
      true, // Whether to mirror path to match alliance
      Drivetrain.getInstance() // Subsystem requirements
    );
  }

  /**
   * Create a complete autonomous PathPlanner-based command group. This will reset the odometry at the begininng of the first path, follow paths, trigger events during path following, and run commands between paths with stop events.
   *
   * <p> Inserting disconnected paths in the list allows for stop events that move the position of the robot separately from the pathplanner control. </p>
   *
   * @param paths PathPlanner paths to follow
   * @param events String-Command map of named commands to run at events. Names may be reused for multiple events. Commands that run at stop events may require the drivetrain, but no others should.
   */
  public static Command getFollowPathplannerWithEventsCommand(List<PathPlannerTrajectory> paths, Map<String, Command> events) {
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

    return builder.fullAuto(paths);
  }
}