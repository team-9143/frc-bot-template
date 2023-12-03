package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Contains auto types, choosers, and compiler. */
public final class AutoSelector {
  /** @return a full auto routine */
  public static Command getAuto() {
    return new InstantCommand();
  }
}