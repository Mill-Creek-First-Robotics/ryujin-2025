package org.steeltalons.subsystems;

import static org.steeltalons.Constants.ArmConstants.kConstraints;
import static org.steeltalons.Constants.ArmConstants.kD;
import static org.steeltalons.Constants.ArmConstants.kG;
import static org.steeltalons.Constants.ArmConstants.kGearing;
import static org.steeltalons.Constants.ArmConstants.kI;
import static org.steeltalons.Constants.ArmConstants.kInverted;
import static org.steeltalons.Constants.ArmConstants.kMaxAngleDegrees;
import static org.steeltalons.Constants.ArmConstants.kMinAngleDegrees;
import static org.steeltalons.Constants.ArmConstants.kP;
import static org.steeltalons.Constants.ArmConstants.kS;
import static org.steeltalons.Constants.ArmConstants.kV;
import static org.steeltalons.Constants.MotorControllers.kArmMotor;
import static org.steeltalons.Constants.MotorControllers.kDefaultNeo550Config;

import org.steeltalons.lib.Util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control the arm.
 */
public class ArmSubsystem extends SubsystemBase {
  private SparkMax arm = new SparkMax(kArmMotor, MotorType.kBrushless);
  private ProfiledPIDController feedbackController = new ProfiledPIDController(kP, kI, kD, kConstraints);
  private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV);

  public ArmSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeo550Config);
    config.encoder
        // rpm to degrees per second
        .velocityConversionFactor(6 / kGearing)
        // rotations to degrees
        .positionConversionFactor(360 / kGearing);
    config.inverted(kInverted);
    arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // default target pose is the starting position
    setTargetPosition(getPosition());
  }

  // --- Public Member Functions -------------------------------------------------

  /**
   * Sets the arm's target position.
   *
   * Use the static constants in
   * {@link org.steeltalons.Constants.ArmConstants.ArmPositions} for most
   * positions. Units are in rotations.
   *
   * @param pos The desired position in degrees.
   */
  public void setTargetPosition(double pos) {
    feedbackController.reset(getPosition());
    feedbackController.setGoal(pos);
  }

  /**
   * Shift the arm's target position by the provided delta.
   *
   * @param delta the desired change in position in degrees.
   * @return A {@link Command} that runs once to set this subsystem's setpoint.
   *         Does not require the ArmSubsystem.
   */
  public Command changePositionBy(double delta) {
    return Commands.runOnce(() -> setTargetPosition(getPosition() + delta));
  }

  /**
   * Returns a {@link Command} that moves the elevator to the target position.
   * Does not end unless cancelled or interrupted.
   */
  public Command moveToTargetPosition() {
    return run(() -> {
      double fbVolts = feedbackController.calculate(getPosition());
      double ffVolts = feedforwardController.calculate(getPosition(), feedbackController.getSetpoint().velocity);
      setVoltage(fbVolts + ffVolts);
    });
  }

  /**
   * Sets the voltage on the arm motor while considering hard limits.
   *
   * Prefer to call {@link ArmSubsystem#moveToTargetPosition}, but this can be
   * used for manual control if needed.
   */
  public void setVoltage(double volts) {
    double input = Util.clamp(volts, -12, 12);
    double currentPos = getPosition();

    if (currentPos > kMaxAngleDegrees || currentPos < kMinAngleDegrees) {
      input = 0;
    }

    arm.setVoltage(input);
  }

  /**
   * Returns true when the arm is at its target position.
   */
  public boolean atSetpoint() {
    return feedbackController.atSetpoint();
  }

  // --- Private Member Functions ------------------------------------------------

  private double getPosition() {
    return arm.getEncoder().getPosition();
  }

  // --- SubsystemBase -----------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position (deg)", this::getPosition, null);
    builder.addDoubleProperty("setpoint (deg)", () -> feedbackController.getSetpoint().position, null);
    builder.addDoubleProperty("velocity (dps)", arm.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("current", arm::getOutputCurrent, null);
    builder.addDoubleProperty("output volts", () -> arm.get() * 12, null);
  }
}
