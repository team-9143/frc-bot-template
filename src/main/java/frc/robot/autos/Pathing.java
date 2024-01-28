package frc.robot.autos;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.logger.Logger;

import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.commands.FollowPathHolonomic;
//import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
//import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants.DriveConsts;
import java.util.Map;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.nio.file.Path;
import java.util.List;

/** Utility class for loading, generating, following, and logging paths through PathPlanner. PID controllers are initialized at runtime so that TunableNumbers can take effect. */
public class Pathing {
  private static final String PATH_LOG_DIR = "/pathplanner/";

  /** Default constraints for accurately generating and following paths */
  private static final PathConstraints default_constraints = new PathConstraints(DriveConsts.kMaxLinearVelMetersPerSecond, DriveConsts.kMaxLinearAccelMetersPerSecondSquared, DriveConsts.kMaxTurnVelRadiansPerSecond, DriveConsts.kMaxTurnAccelRadiansPerSecondSquared);

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
    /*DOES NOT WORK */
  // public static PathPlannerTrajectory loadPath(String name, double maxVelMetersPerSecond, double maxAccelMetersPerSecondPerSecond) {
  //   return PathPlannerPath.(name, Math.min(maxVelMetersPerSecond, default_constraints.getMaxVelocityMps()), maxAccelMetersPerSecondPerSecond);
  // }

  /**
   * Load a pathplanner path, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   */
  // public static PathPlannerTrajectory loadPath(String name) {
  //   return loadPath(name, default_constraints.getMaxVelocityMps(), default_constraints.getMaxAccelerationMpsSq());
  // }
  public static PathPlannerPath loadPath(String name) {
    return loadPath(name);

  }

  /**
   * Load a pathplanner path, separated into a list by stop events, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   * @param maxVelMetersPerSecond max velocity along the path. Paths will not exceed the maximum velocity of the robot.
   * @param maxAccelMetersPerSecondPerSecond max acceleration along the path. Use {@code Double.POSITIVE_INFINITY} for immediate starts and stops.
   */
  public static List<PathPlannerTrajectory> loadPathGroup(String name, double maxVelMetersPerSecond, double maxAccelMetersPerSecondPerSecond) {
    return loadPathGroup(name, Math.min(maxVelMetersPerSecond, default_constraints.getMaxVelocityMps()), maxAccelMetersPerSecondPerSecond);
  }

  public static Command loadPath(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  /**
   * Load a pathplanner path, separated into a list by stop events, constrained by the default maximum velocity and acceleration.
   *
   * @param name name of the path file under [deploy/pathplanner/], omitting ".path"
   */
  public static List<PathPlannerTrajectory> loadPathGroup(String name) {
    return loadPathGroup(name, default_constraints.getMaxVelocityMps(), default_constraints.getMaxAccelerationMpsSq());
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
    //PathPoint first = new PathPoint(startPoseMetersCCW.getTranslation(), relativeAngle, startPoseMetersCCW.getRotation());
    //PathPoint last = new PathPoint(endPoseMetersCCW.getTranslation(), relativeAngle, endPoseMetersCCW.getRotation());
    Pose2d first = new Pose2d(startPoseMetersCCW.getTranslation(), startPoseMetersCCW.getRotation());
    Pose2d last = new Pose2d(endPoseMetersCCW.getTranslation(), endPoseMetersCCW.getRotation());


    // Generate the path with the constraints provided (velocity cannot exceed maximum robot velocity)
    //return PathPlanner.generatePath(new PathConstraints(Math.min(maxVelMetersPerSecond, default_constraints.getMaxVelocityMps()), maxAccelMetersPerSecondPerSecond), first, last);
    return generateDirectPath(first, last, Math.min(maxVelMetersPerSecond, default_constraints.getMaxVelocityMps()), maxAccelMetersPerSecondPerSecond);
  }

  /**
   * Create a pathplanner path, constrained by the default maximum velocity and acceleration.
   *
   * @param startPoseMetersCCW starting position for the command
   * @param endPoseMetersCCW ending position for the command
   */
  public static PathPlannerTrajectory generateDirectPath(Pose2d startPoseMetersCCW, Pose2d endPoseMetersCCW) {
    return generateDirectPath(startPoseMetersCCW, endPoseMetersCCW, default_constraints.getMaxVelocityMps(), default_constraints.getMaxAccelerationMpsSq());
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
  public static HolonomicDriveController getFollowPathplannerCommand(PathPlannerTrajectory path) {
    // return new PPSwerveControllerCommand(
    //   path, // Path to follow
    //   Drivetrain.getInstance()::getPose, // Pose supplier
    //   new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
    //   new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
    //   new PIDController(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
    //   speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
    //   true, // Whether to mirror path to match alliance
    //   Drivetrain.getInstance() // Subsystem requirements
    // );
    return new FollowPathHolonomic(path, null, speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), null, null, true, Drivetrain.getInstance());
    }

  /**
   * Create a complete autonomous PathPlanner-based command group. This will reset the odometry at the begininng of the first path, follow paths, trigger events during path following, and run commands between paths with stop events.
   *
   * <p> Inserting disconnected paths in the list allows for stop events that move the position of the robot separately from the pathplanner control. </p>
   *
   * @param paths PathPlanner paths to follow
   * @param events String-Command map of named commands to run at events. Names may be reused for multiple events. Commands that run at stop events may require the drivetrain, but no others should.
   */
  // public static Command getFollowPathplannerWithEventsCommand(List<PathPlannerTrajectory> paths, Map<String, Command> events) {
  //   var builder = new AutoBuilder(
  //     Drivetrain.getInstance()::getPose, // Pose supplier
  //     Drivetrain.getInstance()::resetOdometry, // Pose consumer to set the odometry to the start of the path
  //     new PIDConstants(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
  //     new PIDConstants(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
  //     speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
  //     events, // Commands to run at named events
  //     true, // Whether to mirror path to match alliance
  //     Drivetrain.getInstance() // Subsystem requirements
  //   );

  //   return builder.fullAuto(paths);
  // }
  public static Command getFollowPathplannerWithEventsCommand(){
    AutoBuilder.configureHolonomic(
      Drivetrain.getInstance()::getPose, // Pose supplier
      Drivetrain.getInstance()::resetOdometry, // Pose consumer to set the odometry to the start
      Drivetrain.getInstance()::getMeasuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      speeds -> Drivetrain.getInstance().driveFieldRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), // Field-relative velocities consumer
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), //Translation controller for position error -> velocity
        new PIDConstants(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotationcontroller for angle error -> angular velocity
        4.5, // Max module speed, in m/s
        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      Drivetrain.getInstance() // Subsystem requirements
        );
  }

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
  private static HolonomicPathFollowerConfig getFollowerConfig(ReplanningConfig replanningConfig) {
    return new HolonomicPathFollowerConfig(
      new PIDConstants(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()), // Translation controller for position error -> velocity
      new PIDConstants(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()), // Rotation controller for angle error -> angular velocity
      DriveConsts.kMaxLinearVelMetersPerSecond, // Maximum module speed
      frc.robot.Constants.SwerveConsts.kSwerve_fl.location.getDistance(new Translation2d()), // Radius of drive base
      replanningConfig
    );
  }
}