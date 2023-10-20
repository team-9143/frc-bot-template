// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.util.TunableNumber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotContainer.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    RobotContainer.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // Enable command scheduling in test mode
    CommandScheduler.getInstance().enable();

    var groups = TunableNumber.getAllGroups();

    // Indexed loop keeps timer synchronous
    for (var group : groups) {
      var instances = TunableNumber.getGroup(group);
      var layout = Shuffleboard.getTab("Tunables").getLayout(group, BuiltInLayouts.kList);
      boolean initialized = true;

      for (var elem : instances) {
        // Make all tunables mutable
        elem.setMutable(true);

        // Add tunables to shuffleboard if applicable
        if (!elem.hasEntry()) {
          initialized = false; // Layout has not been fully initialized
          elem.setEntry(
            layout.add(elem.m_name, elem.getAsDouble())
              .withWidget(BuiltInWidgets.kTextView)
              .withSize(2, 2)
              .getEntry()
          );
        }
      }

      if (!initialized) {
        layout.add("Reset", new InstantCommand(() -> {
          for (var elem : instances) {
            elem.reset();
          }
        }));
      }
    }
  }

  @Override
  public void testPeriodic() {
    // Update all tunable numbers
    TunableNumber.updateAll();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}