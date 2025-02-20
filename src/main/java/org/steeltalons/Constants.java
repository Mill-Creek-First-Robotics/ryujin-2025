package org.steeltalons;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
    public static int kFrontLeft = 1;
    public static int kFrontRight = 2;
    public static int kRearLeft = 3;
    public static int kRearRight = 4;
  } // end MotorControllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class DrivetrainConstants {
    private DrivetrainConstants() {
    }

  } // end DrivetrainConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class ElevatorConstants {
    private ElevatorConstants() {
    }

  } // end ElevatorConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  public static class IntakeConstants {
    private IntakeConstants() {
    }

  } // end IntakeConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
