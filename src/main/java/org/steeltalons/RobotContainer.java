package org.steeltalons;

import static org.steeltalons.Constants.kControllerPort;

import org.steeltalons.subsystems.DriveSubsystem;
import org.steeltalons.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(kControllerPort);

  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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
  }

  private void configureBindings() {
    controller.rightBumper().whileTrue(intakeSubsystem.runIntake());
    controller.leftBumper().whileTrue(intakeSubsystem.reverseIntake());
  }

  public Command getAutonomousCommand() {
    return Autos.passLine(driveSubsystem);
  }
}
