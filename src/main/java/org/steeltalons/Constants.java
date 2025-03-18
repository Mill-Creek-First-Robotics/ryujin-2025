package org.steeltalons;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class Constants {
  private Constants() {
  }

  public static final int kControllerPort = 0;

  public static class MotorControllers {
    private MotorControllers() {
    }

    // [Default Configs]
    public static final SparkBaseConfig kDefaultNeoConfig = new SparkMaxConfig()
        .voltageCompensation(12)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80);
    public static final SparkBaseConfig kDefaultNeo550Config = new SparkMaxConfig()
        .apply(kDefaultNeoConfig)
        .smartCurrentLimit(20);

    // [Drivetrain]
    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kRearLeft = 3;
    public static final int kRearRight = 4;
  } // end MotorControllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class DrivetrainConstants {
    private DrivetrainConstants() {
    }

    public static final double kGearRatio = 10.71;
    // [Dimensions]
    public static final Distance kWheelDiameter = Units.Inches.of(6);
    // horizontal distance from middle to a wheel in meters
    public static final double kDriveTrainBaseX = .212725; // placeholder
    // vertical distance from middle to a wheel in meters
    public static final double kDrivetrainBaseY = .2667; // placeholder
    public static final Translation2d[] kWheelOffsets = new Translation2d[] {
        new Translation2d(kDriveTrainBaseX, kDrivetrainBaseY),
        new Translation2d(kDriveTrainBaseX, -kDrivetrainBaseY),
        new Translation2d(-kDriveTrainBaseX, kDrivetrainBaseY),
        new Translation2d(-kDriveTrainBaseX, -kDrivetrainBaseY),
    };
    // [Physics stuff]
    public static final Mass kMass = Units.Kilograms.of(34); // placeholder (taken from CAD)
    public static final MomentOfInertia kMomentOfInertia = Units.KilogramSquareMeters.of(3.35); // placeholder
    public static final double kWheelCoefficientOfFriction = .7;
    public static final LinearVelocity kMaxSpeed = Units.FeetPerSecond.of(14.5); // placeholder
    public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
        kWheelOffsets[0],
        kWheelOffsets[1],
        kWheelOffsets[2],
        kWheelOffsets[3]);
  } // end DrivetrainConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
