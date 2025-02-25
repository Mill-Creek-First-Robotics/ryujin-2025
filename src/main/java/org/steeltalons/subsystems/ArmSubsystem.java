package org.steeltalons.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.steeltalons.Constants.kTuningModeEnabled;
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

import org.steeltalons.lib.TunableNumber;
import org.steeltalons.lib.Util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/**
 * Subsystem to control the arm.
 */
public class ArmSubsystem extends SubsystemBase {
  private SparkMax arm = new SparkMax(kArmMotor, MotorType.kBrushless);
  private ProfiledPIDController feedbackController = new ProfiledPIDController(kP, kI, kD, kConstraints);
  private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV);

  private final TunableNumber p = new TunableNumber("ArmSubsystem/P", kP);
  private final TunableNumber d = new TunableNumber("ArmSubsystem/D", kD);
  private final TunableNumber g = new TunableNumber("ArmSubsystem/G", kG);
  private final TunableNumber v = new TunableNumber("ArmSubsystem/V", kV);
  private final TunableNumber s = new TunableNumber("ArmSubsystem/S", kS);

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
      double ffVolts = feedforwardController.calculate(Units.degreesToRadians(getPosition()),
          feedbackController.getSetpoint().velocity);
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

  /**
   * Returns the arm's SysIdRoutine.
   */
  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
        new Config(
            Volts.of(1).per(Second), Volts.of(3), Seconds.of(2)),
        new Mechanism(
            volts -> setVoltage(volts.magnitude()),
            log -> {
              log.motor("arm")
                  .voltage(Volts.of(arm.getAppliedOutput() * 12))
                  .angularPosition(Degrees.of(getPosition()))
                  .angularVelocity(DegreesPerSecond.of(arm.getEncoder().getVelocity()));
            }, this));
  }

  // --- Private Member Functions ------------------------------------------------

  private double getPosition() {
    return arm.getEncoder().getPosition();
  }

  // --- SubsystemBase -----------------------------------------------------------

  @Override
  public void periodic() {
    if (kTuningModeEnabled) {
      if (p.hasChanged())
        feedbackController.setP(p.get());
      if (d.hasChanged())
        feedbackController.setD(d.get());
      // ArmFeedforward has no setter functions, so it must be reassigned
      if (g.hasChanged() || v.hasChanged() || s.hasChanged())
        feedforwardController = new ArmFeedforward(s.get(), g.get(), v.get());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position (deg)", this::getPosition, null);
    builder.addDoubleProperty("setpoint (deg)", () -> feedbackController.getSetpoint().position, null);
    builder.addDoubleProperty("velocity (dps)", arm.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("current", arm::getOutputCurrent, null);
    builder.addDoubleProperty("output volts", () -> arm.get() * 12, null);
  }
}
