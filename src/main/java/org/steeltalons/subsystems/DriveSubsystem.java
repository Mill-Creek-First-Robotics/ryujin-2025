package org.steeltalons.subsystems;

import static org.steeltalons.Constants.MotorControllers.kDefaultNeoConfig;
import static org.steeltalons.Constants.MotorControllers.kFrontLeft;
import static org.steeltalons.Constants.MotorControllers.kFrontRight;
import static org.steeltalons.Constants.MotorControllers.kRearLeft;
import static org.steeltalons.Constants.MotorControllers.kRearRight;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the drivetrain.
 */
public class DriveSubsystem extends SubsystemBase {
  private SparkMax flMotor = new SparkMax(kFrontLeft, MotorType.kBrushless);
  private SparkMax frMotor = new SparkMax(kFrontRight, MotorType.kBrushless);
  private SparkMax rlMotor = new SparkMax(kRearLeft, MotorType.kBrushless);
  private SparkMax rrMotor = new SparkMax(kRearRight, MotorType.kBrushless);
  private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  
  public DriveSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeoConfig);

    flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rlMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // inverse right side of the drivetrain
    config.inverted(true);
    frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rrMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // --- Public Member Functions ------------------------------------------------------------------
  
  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured counterclockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param x The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param y The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
   * @param z The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   * @param isFieldOriented Determines whether to drive the robot relative to the field, or to itself.
   */
  public void driveCartesian(double x, double y, double z, boolean isFieldOriented) {
    Rotation2d heading = new Rotation2d();
    if (isFieldOriented) {
      heading = gyro.getRotation2d().unaryMinus();
    }
    drivetrain.driveCartesian(x, y, z, heading);
  }
}
