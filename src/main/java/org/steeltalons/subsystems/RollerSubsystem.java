package org.steeltalons.subsystems;

import static org.steeltalons.Constants.MotorControllers.kRollerMotor;
import static org.steeltalons.Constants.RollerConstants.kEjectVolts;

import org.steeltalons.lib.Util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * {@link SubsystemBase} that controls the roller motor.
 */
public class RollerSubsystem extends SubsystemBase {
  private SparkMax roller = new SparkMax(kRollerMotor, MotorType.kBrushed);

  /**
   * Create an instance of the RolleySubsystem. Only one instance can exist at a time.
   */
  public RollerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .voltageCompensation(10)
      .smartCurrentLimit(60);
    roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the voltage on the roller motor.
   * @param volts the voltage to supply to the motor. Will be clamped to the range [-12, 12].
   */
  public void setVoltage(double volts) {
    var input = Util.clamp(volts, -12, 12);
    roller.setVoltage(input);
  }

  /**
   * Returns a {@link Command} that ejects the coral.
   * @return A command requiring this subsystem that will not stop until interrupted.
   */
  public Command eject() {
    return run(() -> setVoltage(kEjectVolts));
  }
}
