package frc.robot.util;

import com.choreo.lib.Choreo;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConsts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class Pathing {
  /** Utility method to retrieve an easy direct move command. Would not recommend using this over Choreo.
   *
   * @param desiredPoseMetersCCW robot pose relative to the same origin as the odometry (UNIT: meters, ccw native angle)
   */
  public static Command getDirectMoveCommand(Pose2d desiredPoseMetersCCW) {
    return Drivetrain.getInstance().run(() -> Drivetrain.getInstance().driveToLocation(desiredPoseMetersCCW, DriveConsts.kMaxWheelVelMetersPerSecond));
  }

  /**
   * Create a follow command for a premade Choreo trajectory.
   *
   * @param name corresponding trajectory file name under [deploy/choreo/], omitting ".traj"
   * @param matchAlliance {@code true} to mirror the path if on the red alliance
   */
  public static Command followChoreoTrajectory(String name, boolean matchAlliance) {
    return Choreo.choreoSwerveCommand(
      Choreo.getTrajectory(name), // Trajectory from deploy directory
      Drivetrain.getInstance()::getPose, // Pose supplier, needs to be field-relative
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()),
      new PIDController(DriveConsts.kTranslateP.getAsDouble(), DriveConsts.kTranslateI.getAsDouble(), DriveConsts.kTranslateD.getAsDouble()),
      new PIDController(DriveConsts.kRotateP.getAsDouble(), DriveConsts.kRotateI.getAsDouble(), DriveConsts.kRotateD.getAsDouble()),
      speeds -> Drivetrain.getInstance().driveRobotRelativeVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond),
      matchAlliance,
      Drivetrain.getInstance()
    );
  }
}