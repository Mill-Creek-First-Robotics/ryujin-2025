package org.steeltalons;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class Constants {
  private Constants() {
  }

  public static final boolean kTuningModeEnabled = true;
  public static final boolean kSysIdModeEnabled = true;
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

    // [Drivetrain] // placeholders
    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kRearLeft = 3;
    public static final int kRearRight = 4;
    // [Elevator] // placeholder
    public static final int kElevatorMotor = 5;
    // [Arm] // placeholder
    public static final int kArmMotor = 6;
    // [Intake] // placeholder
    public static final int kIntakeMotor = 7;
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

  public static class ElevatorConstants {
    private ElevatorConstants() {
    }

    /**
     * Contains static constants that represent all the different elevator
     * positions. Units are in Rotations.
     */
    public static class ElevatorPositions {
      public static final double kBottom = 0.0698;
      public static final double kIntakePrep = 0.55;
      public static final double kIntake = 0.355;
      public static final double kAlgaeL2 = 0.884;
      public static final double kAlgaeL3 = 1.234;
      public static final double kL1 = 0.323;
      public static final double kL2 = 0.31;
      public static final double kL3 = 0.7;
      public static final double kL4 = 1.27;
      public static final double kTop = 1.57;
    }

    // [Feedback]
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    // [Feedforward]
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kS = 0;

    // change (in meters) required to score on certain reef levels
    public static final double kScoringMovement = -0.25;

    // [Limits]
    public static final double kMaxHeight = 2;
    public static final double kMinHeight = 0;

    public static final double kMaxVelocityMetersPerSecond = 1.3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
        kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
  } // end ElevatorConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class ArmConstants {
    private ArmConstants() {
    }

    /**
     * Contains static constants that represent all the different arm positions.
     * Units are in degrees. Zero is horizontal. Counterclockwise (if looking at the
     * robot where the arm is facing to the right) is positive.
     */
    public static class ArmPositions {
      public static final double kBottom = -90;
      public static final double kHorizontal = 0;
      public static final double kL1 = 0;
      public static final double kL2 = 55;
      public static final double kL3 = 55;
      public static final double kL4 = 59.18654;
      public static final double kTop = 90;
    }

    public static final boolean kInverted = true;
    // 20:1 gearbox + 2:1 pulleys
    public static final double kGearing = 40;

    // change (in degrees) required to score on certain reef levels
    public static final double kScoringMovement = -45.84;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 0.0;

    public static final double kMaxAngleDegrees = 95;
    public static final double kMinAngleDegrees = -100;

    public static final double kMaxVelocityDegreesPerSecond = 270;
    public static final double kMaxAccelerationDegreesPerSecondSquared = 720;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
        kMaxVelocityDegreesPerSecond, kMaxAccelerationDegreesPerSecondSquared);
  } // end ArmConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class IntakeConstants {
    private IntakeConstants() {
    }

    public static final boolean kInverted = false;
    public static final double kIntakeVoltage = 3;
    public static final double kOuttakeVoltage = -12;

  } // end IntakeConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
