package org.steeltalons;

import static org.steeltalons.Constants.kControllerPort;

import org.steeltalons.subsystems.DriveSubsystem;
import org.steeltalons.subsystems.RollerSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(kControllerPort);

  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();

    SmartDashboard.putData(driveSubsystem);

    NamedCommands.registerCommand("roller_eject", rollerSubsystem.eject());
    NamedCommands.registerCommand("roller_stop", rollerSubsystem.stop());
    NamedCommands.registerCommand("score", Commands.sequence(
      rollerSubsystem.eject().withTimeout(0.3),
      // schedule command is used so that this command composition ends instantly, but the motor stays stopped.
      new ScheduleCommand(rollerSubsystem.stop())
    ));

    autoChooser = AutoBuilder.buildAutoChooser("Pass line");
    autoChooser.addOption("Pass line", Autos.passLine(driveSubsystem));
    autoChooser.addOption("NOTHING", Commands.none());
    SmartDashboard.putData("Auto chooser", autoChooser);
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
    return autoChooser.getSelected();
  }
}
