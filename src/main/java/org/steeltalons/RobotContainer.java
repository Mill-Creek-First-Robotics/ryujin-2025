// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steeltalons;

import static org.steeltalons.Constants.kControllerPort;

import org.steeltalons.subsystems.ArmSubsystem;
import org.steeltalons.subsystems.DriveSubsystem;
import org.steeltalons.subsystems.ElevatorSubsystem;
import org.steeltalons.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(kControllerPort);

  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private ArmSubsystem armSubsystem = new ArmSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();

    SmartDashboard.putData(elevatorSubsystem);
    SmartDashboard.putData(armSubsystem);
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
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.run(() -> elevatorSubsystem.setVoltage(0)));
    armSubsystem.setDefaultCommand(armSubsystem.run(() -> armSubsystem.setVoltage(0)));
  }

  private void configureBindings() {
    controller.rightTrigger(0.1)
        .whileTrue(elevatorSubsystem.run(() -> elevatorSubsystem.setVoltage(controller.getRightTriggerAxis() * 4)));
    controller.leftTrigger(0.1)
        .whileTrue(elevatorSubsystem.run(() -> elevatorSubsystem.setVoltage(-controller.getLeftTriggerAxis() * 4)));

    controller.povUp().whileTrue(armSubsystem.run(() -> armSubsystem.setVoltage(8)));
    controller.povDown().whileTrue(armSubsystem.run(() -> armSubsystem.setVoltage(-8)));

    controller.rightBumper().whileTrue(intakeSubsystem.runIntake());
    controller.leftBumper().whileTrue(intakeSubsystem.reverseIntake());
  }

  public Command getAutonomousCommand() {
    return Autos.passLine(driveSubsystem);
  }
}
