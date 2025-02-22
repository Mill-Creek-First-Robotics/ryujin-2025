package org.steeltalons.subsystems;

import static org.steeltalons.Constants.IntakeConstants.kIntakeVoltage;
import static org.steeltalons.Constants.IntakeConstants.kInverted;
import static org.steeltalons.Constants.IntakeConstants.kOuttakeVoltage;
import static org.steeltalons.Constants.MotorControllers.kDefaultNeo550Config;
import static org.steeltalons.Constants.MotorControllers.kIntakeMotor;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to interact with the intake.
 */
public class IntakeSubsystem extends SubsystemBase {
  private SparkMax intake = new SparkMax(kIntakeMotor, MotorType.kBrushless);

  public IntakeSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeo550Config);
    config.inverted(kInverted);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // --- Public Member Functions -------------------------------------------------

  /**
   * Returns a {@link Command} that runs ths intake forwards. Will run until
   * cancelled or interrupted.
   */
  public Command runIntake() {
    return Commands.startEnd(
        () -> intake.setVoltage(kIntakeVoltage),
        () -> intake.stopMotor()).withName("Intake.runIntake");
  }

  /**
   * Returns a {@link Command} that runs the intake backwards. Will run until
   * cancelled or interrupted.
   */
  public Command reverseIntake() {
    return Commands.startEnd(
        () -> intake.setVoltage(kOuttakeVoltage),
        () -> intake.stopMotor()).withName("Intake.reverseIntake");
  }
}
