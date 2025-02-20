package org.steeltalons.subsystems;

import static org.steeltalons.Constants.ElevatorConstants.kConstraints;
import static org.steeltalons.Constants.ElevatorConstants.kD;
import static org.steeltalons.Constants.ElevatorConstants.kG;
import static org.steeltalons.Constants.ElevatorConstants.kI;
import static org.steeltalons.Constants.ElevatorConstants.kMaxHeight;
import static org.steeltalons.Constants.ElevatorConstants.kMinHeight;
import static org.steeltalons.Constants.ElevatorConstants.kP;
import static org.steeltalons.Constants.ElevatorConstants.kS;
import static org.steeltalons.Constants.ElevatorConstants.kV;
import static org.steeltalons.Constants.MotorControllers.kDefaultNeo550Config;
import static org.steeltalons.Constants.MotorControllers.kElevatorMotor;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax motor = new SparkMax(kElevatorMotor, MotorType.kBrushless);
  private ProfiledPIDController feedbackController = new ProfiledPIDController(kP, kI, kD, kConstraints);
  private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV);

  public ElevatorSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeo550Config);
    config.encoder
        .positionConversionFactor(1d)
        .velocityConversionFactor(1d);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // default target pose is the starting position
    setTargetPosition(getPosition());
  }

  // --- Public Member Functions -------------------------------------------------

  /**
   * Sets the elevator's target position.
   *
   * Use the static constants in
   * {@link org.steeltalons.Constants.ElevatorConstants.Positions} for most
   * positions. Units are in rotations.
   *
   * @param pos The desired position in rotations.
   */
  public void setTargetPosition(double pos) {
    feedbackController.reset(getPosition());
    feedbackController.setGoal(pos);
  }

  /**
   * Returns a {@link Command} that moves the elevator to the target position.
   * Does not end unless cancelled or interrupted.
   */
  public Command moveToTargetPosition() {
    return run(() -> {
      double fbVolts = feedbackController.calculate(getPosition());
      double ffVolts = feedforwardController.calculate(feedbackController.getSetpoint().velocity);
      setVoltage(fbVolts + ffVolts);
    }).withName("Elevator.moveToTargetPosition");
  }

  /**
   * Sets the voltage on the elevator motor while considering hard limits.
   *
   * Prefer to call {@link ElevatorSubsystem.moveToTargetPosition}, but this can
   * be used for manual control if needed.
   */
  public void setVoltage(double volts) {
    double input = Math.clamp(volts, -12, 12);
    double currentPos = getPosition();

    if (currentPos > kMaxHeight || currentPos < kMinHeight) {
      input = 0;
    }

    motor.setVoltage(input);
  }

  // --- Private Member Functions ------------------------------------------------

  private double getPosition() {
    return motor.getEncoder().getPosition();
  }
}
