package org.steeltalons;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    // [Drivetrain] // placeholders
    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kRearLeft = 3;
    public static final int kRearRight = 4;
    // [Elevator] // placeholder
    public static final int kElevatorMotor = 5;
  } // end MotorControllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class DrivetrainConstants {
    private DrivetrainConstants() {
    }

  } // end DrivetrainConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class ElevatorConstants {
    private ElevatorConstants() {
    }

    /**
     * Contains static constants that represent all the different elevator
     * positions. Units are in Rotations.
     */
    public static class Positions {
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
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 1;
    // [Feedforward]
    public static final double kG = 1;
    public static final double kV = 1;
    public static final double kS = 1;

    // [Limits]
    public static final double kMaxHeight = 20;
    public static final double kMinHeight = 0;

    public static final double kMaxVelocityMetersPerSecond = 1.3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final TrapezoidProfile.Constraints kConstraints = new TrapezoidProfile.Constraints(
        kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
  } // end ElevatorConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class IntakeConstants {
    private IntakeConstants() {
    }

  } // end IntakeConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
