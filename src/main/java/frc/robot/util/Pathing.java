package frc.robot.util;

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
}