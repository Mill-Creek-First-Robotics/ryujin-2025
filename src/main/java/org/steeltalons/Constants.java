package org.steeltalons;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.Amps;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
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
    // [Roller]
    public static final int kRollerMotor = 5;
  } // end MotorControllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class DrivetrainConstants {
    private DrivetrainConstants() {
    }

    public static final double kGearRatio = 10.71;
    public static final Current kMaxCurrent = Amps.of(50);
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
    // --- PID ---------------------------------------------------------------------
    // [Pathplanning]
    public static final double kTranslationalXP = 5; // placeholder
    public static final double kTranslationalYP = 5; // placeholder
    public static final double kRotationalP = 5; // placeholder
    // [Motors]
    public static final double kP = 0.002; // placeholder
    public static final double kD = 0; // placeholder

    // --- Pathplanner -------------------------------------------------------------
    public static final RobotConfig kRobotConfig = new RobotConfig(
        kMass,
        kMomentOfInertia,
        new ModuleConfig(
            kWheelDiameter.times(.5),
            kMaxSpeed,
            kWheelCoefficientOfFriction,
            DCMotor.getNEO(1).withReduction(kGearRatio),
            kMaxCurrent,
            1),
        kWheelOffsets);
    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
        new PIDConstants(kTranslationalXP),
        new PIDConstants(kRotationalP));
  } // end DrivetrainConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class RollerConstants {
    private RollerConstants() {
    }

    public static final double kEjectVolts = 5.28;
  }

  public static class VisionConstants {
    public static final String kCameraName = "limelight";
    // relative to center of robot
    public static final Transform3d kCameraPos = new Transform3d(
        // 0.1m forwards, 0.5m up
        new Translation3d(0.1, 0.0, 0.5),
        // tilted 15deg up
        new Rotation3d(0, degreesToRadians(-15), 0));
    // the standard deviations of vision estimated poses that affect correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); // placeholder
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // placeholder
  }
}
