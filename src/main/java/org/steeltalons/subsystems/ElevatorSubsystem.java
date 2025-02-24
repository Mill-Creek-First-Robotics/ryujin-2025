package org.steeltalons.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.steeltalons.Constants.kTuningModeEnabled;
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

import org.steeltalons.lib.TunableNumber;
import org.steeltalons.lib.Util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/**
 * Subsystem for controlling the elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax motor = new SparkMax(kElevatorMotor, MotorType.kBrushless);
  private ProfiledPIDController feedbackController = new ProfiledPIDController(kP, kI, kD, kConstraints);
  private ElevatorFeedforward feedforwardController = new ElevatorFeedforward(kS, kG, kV);

  private final TunableNumber p = new TunableNumber("ElevatorSubsystem/P", kP);
  private final TunableNumber d = new TunableNumber("ElevatorSubsystem/D", kD);
  private final TunableNumber g = new TunableNumber("ElevatorSubsystem/G", kG);
  private final TunableNumber v = new TunableNumber("ElevatorSubsystem/V", kV);
  private final TunableNumber s = new TunableNumber("ElevatorSubsystem/S", kS);

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
   * Shift the elevator's target position by the provided delta.
   *
   * @param delta the desired change in position in meters.
   * @return A {@link Command} that runs once to set this subsystem's setpoint.
   *         Does not require the ElevatorSubsystem.
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
      double ffVolts = feedforwardController.calculate(feedbackController.getSetpoint().velocity);
      setVoltage(fbVolts + ffVolts);
    }).withName("Elevator.moveToTargetPosition");
  }

  /**
   * Sets the voltage on the elevator motor while considering hard limits.
   *
   * Prefer to call {@link ElevatorSubsystem#moveToTargetPosition}, but this can
   * be used for manual control if needed.
   */
  public void setVoltage(double volts) {
    double input = Util.clamp(volts, -12, 12);
    double currentPos = getPosition();

    if (currentPos > kMaxHeight || currentPos < kMinHeight) {
      input = 0;
    }

    motor.setVoltage(input);
  }

  /**
   * Returns true when the elevator is at its target position.
   */
  public boolean atSetpoint() {
    return feedbackController.atSetpoint();
  }

  public SysIdRoutine getSysIdRoutine() {
    return new SysIdRoutine(
        new Config(
            Volts.of(1).per(Second), Volts.of(5), Seconds.of(3)),
        new Mechanism(
            volts -> setVoltage(volts.magnitude()),
            log -> {
              log.motor("elevator")
                  .voltage(Volts.of(motor.getAppliedOutput() * 12))
                  .linearPosition(Meters.of(getPosition()))
                  .linearVelocity(MetersPerSecond.of(motor.getEncoder().getVelocity()));
            }, this));
  }

  // --- Private Member Functions ------------------------------------------------

  private double getPosition() {
    return motor.getEncoder().getPosition();
  }

  // --- SubsystemBase -----------------------------------------------------------

  @Override
  public void periodic() {
    if (kTuningModeEnabled) {
      if (p.hasChanged())
        feedbackController.setP(p.get());
      if (d.hasChanged())
        feedbackController.setD(d.get());
      // ElevatorFeedforward has no setter functions, so it must be reassigned
      if (g.hasChanged() || v.hasChanged() || s.hasChanged())
        feedforwardController = new ElevatorFeedforward(s.get(), g.get(), v.get());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("position (m)", this::getPosition, null);
    builder.addDoubleProperty("setpoint (m)", () -> feedbackController.getSetpoint().position, null);
    builder.addDoubleProperty("velocity (mps)", motor.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("current", motor::getOutputCurrent, null);
    builder.addDoubleProperty("output volts", () -> motor.get() * 12, null);
  }
}
