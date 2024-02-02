package frc.robot.autos;

import frc.robot.subsystems.Drivetrain;
import frc.robot.logger.Logger;

import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.AutoConsts;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;

/** Utility class for loading, generating, following, and logging paths through PathPlanner. PID controllers are initialized at runtime so that TunableNumbers can take effect. */
public class Pathing {
  private static final String PATH_LOG_DIR = "/pathplanner/";

  /** Default constraints for accurately generating and following paths */
  private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(DriveConsts.kMaxLinearVelMetersPerSecond, AutoConsts.kMaxLinearAccelMetersPerSecondSquared, DriveConsts.kMaxTurnVelRadiansPerSecond, AutoConsts.kMaxTurnAccelRadiansPerSecondSquared);

  // Set up logging for basic path following
  static {
    // Log trajectory on command initialization
    PathPlannerLogging.setLogActivePathCallback(poses ->
      Logger.recordOutput(PATH_LOG_DIR+"activePath", poses.toArray(Pose2d[]::new))
    );

    // No need to log current robot pose, drivetrain will do that
    PathPlannerLogging.setLogCurrentPoseCallback(null);

    // Log reference pose during command run
    PathPlannerLogging.setLogTargetPoseCallback(pose ->
      Logger.recordOutput(PATH_LOG_DIR+"referencePose", pose)
    );
  }

  // Configure AutoBuilder
  static {
    AutoBuilder.configureHolonomic(
      Drivetrain.getInstance()::getPose, // Pose supplier
      Drivetrain.getInstance()::resetOdometry, // Reset pose consumer
      Drivetrain.getInstance()::getDesiredSpeeds, // Current measured speeds
      Drivetrain.getInstance()::driveFieldRelativeVelocity, // Drives field relative from ChassisSpeeds
      getHolonomicConfig(new ReplanningConfig(false, false)), // Config
      Pathing::isRedAlliance, // Flip if alliance is red
      Drivetrain.getInstance() // Subsystem
    );
  }

  /******
   *
   * Path loading and generation
   *
   ******/

  /**
   * Load a pathplanner path from RIO local storage
   *
   * @param name name of the path file under [deploy/pathplanner/paths/], omitting ".path"
   */
  public static PathPlannerPath loadPath(String name) {
    return PathPlannerPath.fromPathFile(name);
  }

  /**
   * Load a choreo trajectory from RIO local storage
   *
   * @param name name of the path file under [deploy/choreo/], omitting ".traj"
   */
  public static PathPlannerPath loadChoreoTraj(String name) {
    return PathPlannerPath.fromChoreoTrajectory(name);
  }

  // TODO: Path gen obstacles in Constants? (stage)
  /**
   * Create a pathplanner path to move directly from one position to another
   *
   * @param startPoseMetersCCW starting position for the command
   * @param endPoseMetersCCW ending position for the command
   */
  public static PathPlannerPath generateDirectPath(Pose2d startPoseMetersCCW, Pose2d endPoseMetersCCW) {
    return new PathPlannerPath(
      // TODO: Test this path generation, start post/end pose rotations may need to be changed to match with bezier curve
      PathPlannerPath.bezierFromPoses(startPoseMetersCCW, endPoseMetersCCW), // Create bezier points from waypoints
      DEFAULT_CONSTRAINTS, // Default constraints
      new GoalEndState(0, endPoseMetersCCW.getRotation()) // End with 0 velocity at specified rotation
    );
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
   * @param replanningConfig replanning configuration to use
   */
  public static FollowPathHolonomic getHolonomicFollowPathCommand(PathPlannerPath path, ReplanningConfig replanningConfig) {
    return new FollowPathHolonomic(
      path, // Path to follow
      Drivetrain.getInstance()::getPose, // Pose supplier
      Drivetrain.getInstance()::getMeasuredSpeeds, // Chassis speeds supplier
      Drivetrain.getInstance()::driveRobotRelativeVelocity, // Robot-relative velocities consumer
      getHolonomicConfig(replanningConfig), // Follower configuration
      Pathing::isRedAlliance, // Flip the path if alliance is red
      Drivetrain.getInstance() // Subsystem requirements
    );
  }

  // TODO: Implement auto building here

  /******
   *
   * Utility
   *
   ******/

  /**
   * Utility method to get a path follower config for the swerve drivetrain
   *
   * @param replanningConfig replanning configuration to use
   * @return the configuration
   */
  private static HolonomicPathFollowerConfig getHolonomicConfig(ReplanningConfig replanningConfig) {
    return new HolonomicPathFollowerConfig(
      new PIDConstants(AutoConsts.kTranslateP.getAsDouble(), AutoConsts.kTranslateI.getAsDouble(), AutoConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDConstants(AutoConsts.kRotateP.getAsDouble(), AutoConsts.kRotateI.getAsDouble(), AutoConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      DriveConsts.kMaxLinearVelMetersPerSecond, // Maximum module speed
      frc.robot.Constants.SwerveConsts.kSwerve_fl.location.getDistance(new Translation2d()), // Radius of drive base
      replanningConfig // When to replan the path
    );
  }

  /**
   * Utility method to check driver station alliance
   *
   * @return {@code true} if alliance is red and not null
   */
  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }
}