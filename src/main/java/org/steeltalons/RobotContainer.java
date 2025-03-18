package org.steeltalons;

import static org.steeltalons.Constants.kControllerPort;

import org.steeltalons.subsystems.DriveSubsystem;
import org.steeltalons.subsystems.RollerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(kControllerPort);

  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private RollerSubsystem rollerSubsystem = new RollerSubsystem();

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.run(() -> {
          driveSubsystem.driveCartesian(
              -controller.getLeftY(),
              controller.getLeftX(),
              controller.getRightX(),
              true);
        }));
    rollerSubsystem.setDefaultCommand(
        rollerSubsystem.run(() -> {
          rollerSubsystem.setVoltage(12 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
        }));
  }

  private void configureBindings() {
    controller.a().whileTrue(rollerSubsystem.eject());
  }

  public Command getAutonomousCommand() {
    return Autos.passLine(driveSubsystem);
  }
}
