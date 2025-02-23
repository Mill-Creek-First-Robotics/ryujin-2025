package org.steeltalons;

import org.steeltalons.Constants.ArmConstants;
import org.steeltalons.Constants.ArmConstants.ArmPositions;
import org.steeltalons.Constants.ElevatorConstants;
import org.steeltalons.Constants.ElevatorConstants.ElevatorPositions;
import org.steeltalons.subsystems.ArmSubsystem;
import org.steeltalons.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Container for the robot's commands.
 *
 * Keeps track of the current target level for scoring purposes.
 */
public class RobotCommands {
  private static Level targetLevel;

  private RobotCommands() {
  }

  private static Command waitUntilAtSetpoint(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return Commands.waitUntil(() -> arm.atSetpoint() && elevator.atSetpoint());
  }

  // --- Coral Scoring -----------------------------------------------------------

  /**
   * Returns a {@link Command} that prepares the elevator and arm to receive and
   * pickup the coral.
   *
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command prepareCoralIntake(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return Commands.runOnce(() -> {
      arm.setTargetPosition(ArmPositions.kBottom);
      elevator.setTargetPosition(ElevatorPositions.kIntakePrep);
    }).andThen(waitUntilAtSetpoint(arm, elevator));
  }

  /**
   * Returns a {@link Command} that picks up the coral with the arm, moves it to
   * the top, and moves the elevator up a bit.
   *
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command intakeCoral(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return Commands.sequence(
        // pickup the coral
        Commands.runOnce(() -> {
          arm.setTargetPosition(ArmPositions.kBottom);
          elevator.setTargetPosition(ElevatorPositions.kBottom);
        }),
        waitUntilAtSetpoint(arm, elevator),
        // move elevator up so arm can swing up.
        Commands.runOnce(() -> {
          elevator.setTargetPosition(ElevatorPositions.kBottom + .31 /* placeholder */);
        }),
        waitUntilAtSetpoint(arm, elevator),
        // then swing arm up and move elevator back down a bit after
        Commands.parallel(
            Commands.runOnce(() -> arm.setTargetPosition(ArmPositions.kTop)),
            Commands.waitSeconds(0.5)
                .andThen(Commands.runOnce(() -> elevator.setTargetPosition(ElevatorPositions.kBottom)))),
        waitUntilAtSetpoint(arm, elevator));
  }

  /**
   * Returns a {@link Command} that prepares the arm and elevator for scoring
   * coral at the provided level.
   *
   * @param level    the {@link Level} to prepare the subsystems for scoring at.
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command prepareScore(Level level, ArmSubsystem arm, ElevatorSubsystem elevator) {
    double elevatorTarget;
    double armTarget;

    switch (level) {
      case kL1 -> {
        elevatorTarget = ElevatorPositions.kL1;
        armTarget = ArmPositions.kL1;
      }
      case kL2 -> {
        elevatorTarget = ElevatorPositions.kL2;
        armTarget = ArmPositions.kL2;
      }
      case kL3 -> {
        elevatorTarget = ElevatorPositions.kL3;
        armTarget = ArmPositions.kL3;
      }
      case kL4 -> {
        elevatorTarget = ElevatorPositions.kL4;
        armTarget = ArmPositions.kL4;
      }
      default -> {
        DriverStation.reportError("Level: " + level.toString() + " Not Found. Picking L1", true);
        elevatorTarget = ElevatorPositions.kL1;
        armTarget = ArmPositions.kL1;
      }
    }

    return Commands.sequence(
        Commands.runOnce(() -> arm.setTargetPosition(armTarget)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> elevator.setTargetPosition(elevatorTarget)),
        waitUntilAtSetpoint(arm, elevator))
        .beforeStarting(() -> targetLevel = level);
  }

  /**
   * Returns a {@link Command} that scores the coral on the current level.
   *
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command score(ArmSubsystem arm, ElevatorSubsystem elevator) {
    Command toRun;
    switch (RobotCommands.targetLevel) {
      case kL1 -> {
        toRun = elevator.changePositionBy(ElevatorConstants.kScoringMovement);
      }
      case kL2 -> {
        toRun = arm.changePositionBy(ArmConstants.kScoringMovement);
      }
      case kL3 -> {
        toRun = arm.changePositionBy(ArmConstants.kScoringMovement);
      }
      case kL4 -> {
        toRun = Commands.parallel(
            arm.changePositionBy(ArmConstants.kScoringMovement),
            Commands.waitSeconds(0.5).andThen(
                elevator.changePositionBy(ElevatorConstants.kScoringMovement)));
      }
      default -> {
        return Commands.none();
      }
    }

    return toRun.andThen(waitUntilAtSetpoint(arm, elevator)).withName("Score");
  }

  // --- Algae Scoring -----------------------------------------------------------

  /**
   * Returns a {@link Command} prepares the robot to remove the L2 algae.
   *
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command prepareAlgaeL2Removal(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return Commands.runOnce(() -> {
      elevator.setTargetPosition(ElevatorPositions.kAlgaeL2);
      arm.setTargetPosition(ArmPositions.kHorizontal);
    }).andThen(waitUntilAtSetpoint(arm, elevator)).withName("prepareAlgaeL2Removal");
  }

  /**
   * Returns a {@link Command} prepares the robot to remove the L3 algae.
   *
   * @param arm      the {@link ArmSubsystem} to control.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command prepareAlgaeL3Removal(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return Commands.runOnce(() -> {
      elevator.setTargetPosition(ElevatorPositions.kAlgaeL3);
      arm.setTargetPosition(ArmPositions.kHorizontal);
    }).andThen(waitUntilAtSetpoint(arm, elevator)).withName("prepareAlgaeL3Removal");
  }

  /**
   * Returns a {@link Command} that removes the algae at the current level.
   *
   * @param arm      the {@link ArmSubsystem} to control. Does not use this
   *                 parameter right now.
   * @param elevator the {@link ElevatorSubsystem} to control.
   * @apiNote Does not require either subsystem so that they can still track their
   *          setpoints with their respective default commands. The command will
   *          end when both subsystems reach their setpoints.
   */
  public static Command removeAlgae(ArmSubsystem arm, ElevatorSubsystem elevator) {
    return elevator.changePositionBy(-0.06).andThen(waitUntilAtSetpoint(arm, elevator)).withName("removeAlgae");
  }
}
