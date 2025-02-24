// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steeltalons;

import static org.steeltalons.Constants.kControllerPort;

import org.steeltalons.subsystems.ArmSubsystem;
import org.steeltalons.subsystems.DriveSubsystem;
import org.steeltalons.subsystems.ElevatorSubsystem;
import org.steeltalons.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
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

    // only log to NetworkTables when not in a match
    if (!DriverStation.isFMSAttached()) {
      SmartDashboard.putData(driveSubsystem);
      SmartDashboard.putData(elevatorSubsystem);
      SmartDashboard.putData(armSubsystem);
      SmartDashboard.putData(intakeSubsystem);
    }
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        driveSubsystem.run(() -> {
          driveSubsystem.driveCartesian(
              -controller.getLeftY(),
              -controller.getLeftX(),
              -controller.getRightX(),
              true);
        }));
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.moveToTargetPosition());
    armSubsystem.setDefaultCommand(armSubsystem.moveToTargetPosition());
  }

  private void configureBindings() {
    controller.povUp().onTrue(RobotCommands.prepareCoralIntake(armSubsystem, elevatorSubsystem));
    controller.povDown().onTrue(RobotCommands.intakeCoral(armSubsystem, elevatorSubsystem));

    controller.x().onTrue(RobotCommands.prepareScore(Level.kL1, armSubsystem, elevatorSubsystem));
    controller.y().onTrue(RobotCommands.prepareScore(Level.kL2, armSubsystem, elevatorSubsystem));
    controller.a().onTrue(RobotCommands.prepareScore(Level.kL3, armSubsystem, elevatorSubsystem));
    controller.b().onTrue(RobotCommands.prepareScore(Level.kL4, armSubsystem, elevatorSubsystem));
    controller.start().onTrue(RobotCommands.score(armSubsystem, elevatorSubsystem));

    controller.povLeft().onTrue(RobotCommands.prepareAlgaeL2Removal(armSubsystem, elevatorSubsystem));
    controller.povRight().onTrue(RobotCommands.prepareAlgaeL3Removal(armSubsystem, elevatorSubsystem));
    controller.leftStick().onTrue(RobotCommands.removeAlgae(armSubsystem, elevatorSubsystem));

    controller.rightBumper().whileTrue(intakeSubsystem.runIntake());
    controller.leftBumper().whileTrue(intakeSubsystem.reverseIntake());
  }

  public Command getAutonomousCommand() {
    return Autos.passLine(driveSubsystem);
  }
}
