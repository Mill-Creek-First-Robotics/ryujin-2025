package org.steeltalons;

import org.steeltalons.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Contains autonomous routines.
 */
public class Autos {
  private Autos() {
  }

  /**
   * Drive the robot past the starting line. Needs TODO meters of space. Runs for 2 seconds. Robot needs to start facing the driver.
   *
   * @param drivetrain the {@link DriveSubsystem} instance to move.
   */
  public static Command passLine(DriveSubsystem drivetrain) {
    return Commands.sequence(
      drivetrain.run(() -> drivetrain.driveCartesian(0.3, 0, 0, false)).withTimeout(2),
      drivetrain.runOnce(() -> drivetrain.driveCartesian(0, 0, 0, false))
    ).withName("Autos.passLine");
  }
}
