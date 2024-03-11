package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.logger.Logger;

/** Contains auto types, choosers, and compiler. */
public class AutoSelector {
  private static final MutableChooser<StartPose> chooser_startPose =
      new MutableChooser<>(StartPose.Wing);

  /** Initializes shuffleboard choosers for auton */
  public static void init() {
    chooser_startPose.setAll(
        StartPose.Subwoofer_front, StartPose.Subwoofer_source_side, StartPose.Subwoofer_amp_side);

    // Add to shuffleboard
    var tab = Shuffleboard.getTab("Auton");
    tab.add("Start pose", chooser_startPose).withPosition(0, 2).withSize(3, 2);

    tab.addBoolean("Reset defaults if red", () -> !chooser_startPose.isUpdateReq())
        .withPosition(0, 0)
        .withSize(4, 2);
  }

  /** Returns a full auto routine */
  public static Command getAuto() {
    var startPose = chooser_startPose.getSelected();

    Logger.log((Pathing.isRedAlliance() ? "RED ALLIANCE" : "BLUE ALLIANCE"));
    Logger.log("AUTON: " + startPose.toString());

    return startPose.getCommand();
  }

  // TODO(user): Create and register named commands here
  static {
    NamedCommands.registerCommand("Empty", new InstantCommand());
  }
}
