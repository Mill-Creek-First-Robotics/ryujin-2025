package org.steeltalons.subsystems;

import static org.steeltalons.Constants.DrivetrainConstants.kD;
import static org.steeltalons.Constants.DrivetrainConstants.kDriveController;
import static org.steeltalons.Constants.DrivetrainConstants.kDriveKinematics;
import static org.steeltalons.Constants.DrivetrainConstants.kGearRatio;
import static org.steeltalons.Constants.DrivetrainConstants.kP;
import static org.steeltalons.Constants.DrivetrainConstants.kRobotConfig;
import static org.steeltalons.Constants.DrivetrainConstants.kWheelDiameter;
import static org.steeltalons.Constants.MotorControllers.kDefaultNeoConfig;
import static org.steeltalons.Constants.MotorControllers.kFrontLeft;
import static org.steeltalons.Constants.MotorControllers.kFrontRight;
import static org.steeltalons.Constants.MotorControllers.kRearLeft;
import static org.steeltalons.Constants.MotorControllers.kRearRight;

import org.steeltalons.Constants.DrivetrainConstants;
import org.steeltalons.Constants.VisionConstants;
import org.steeltalons.lib.LimelightHelpers;
import org.steeltalons.lib.LimelightHelpers.PoseEstimate;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling the drivetrain.
 */
public class DriveSubsystem extends SubsystemBase {
  private SparkMax flMotor = new SparkMax(kFrontLeft, MotorType.kBrushless);
  private SparkMax frMotor = new SparkMax(kFrontRight, MotorType.kBrushless);
  private SparkMax rlMotor = new SparkMax(kRearLeft, MotorType.kBrushless);
  private SparkMax rrMotor = new SparkMax(kRearRight, MotorType.kBrushless);
  private MecanumDrive drivetrain;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // math & telemetry
  private MecanumDrivePoseEstimator poseEstimator;
  // placeholder
  private SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(0, 1 / 473, 0);
  private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Robot Pose", Pose2d.struct).publish();
  private Field2d field = new Field2d();

  public DriveSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeoConfig);
    config.encoder
        // rotations to meters
        .positionConversionFactor(Math.PI * kWheelDiameter.baseUnitMagnitude() / kGearRatio)
        // rpm to meters per second
        .velocityConversionFactor(Math.PI * kWheelDiameter.baseUnitMagnitude() / 60 / kGearRatio);
    config.closedLoop
        .p(kP)
        .d(kD);

    flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rlMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // inverse right side of the drivetrain
    config.inverted(true);
    frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rrMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
    // for pathplanning where the drivetrain's inputs won't be updated
    drivetrain.setSafetyEnabled(false);

    poseEstimator = new MecanumDrivePoseEstimator(DrivetrainConstants.kDriveKinematics, gyro.getRotation2d(),
        getWheelPositions(), new Pose2d());

    // configure pathplanning
    AutoBuilder.configure(
        poseEstimator::getEstimatedPosition,
        poseEstimator::resetPose,
        () -> kDriveKinematics.toChassisSpeeds(getWheelSpeeds()),
        (speeds, ff) -> driveRobotRelative(speeds),
        kDriveController,
        kRobotConfig,
        () -> {
          // flip path only if alliance is red
          if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red;
          }
          return true;
        },
        this);

    // telemetry
    SmartDashboard.putData("Field", field);
    PathPlannerLogging.setLogTargetPoseCallback(pose -> {
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback(poses -> {
      field.getObject("path").setPoses(poses);
    });

    // configure vision
    var cameraPos = VisionConstants.kCameraPos;
    LimelightHelpers.setCameraPose_RobotSpace(
        VisionConstants.kCameraName,
        cameraPos.getX(),
        cameraPos.getY(),
        cameraPos.getZ(),
        cameraPos.getRotation().getX(),
        cameraPos.getRotation().getY(),
        cameraPos.getRotation().getZ());
  }

  // --- Public Member Functions -------------------------------------------------

  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured counterclockwise from the positive X axis. The robot's
   * speed is
   * independent of its angle or rotation rate.
   *
   * @param x               The robot's speed along the X axis [-1.0..1.0].
   *                        Forward is positive.
   * @param y               The robot's speed along the Y axis [-1.0..1.0]. Left
   *                        is positive.
   * @param z               The robot's rotation rate around the Z axis
   *                        [-1.0..1.0]. Counterclockwise is
   *                        positive.
   * @param isFieldOriented Determines whether to drive the robot relative to the
   *                        field, or to itself.
   */
  public void driveCartesian(double x, double y, double z, boolean isFieldOriented) {
    Rotation2d heading = new Rotation2d();
    if (isFieldOriented) {
      heading = gyro.getRotation2d().unaryMinus();
    }
    drivetrain.driveCartesian(x, y, z, heading);
  }

  /**
   * Drive the robot using relative speeds.
   * 
   * @param relativeSpeeds the speeds relative to the robot to use as a setpoint.
   */
  public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
    MecanumDriveWheelSpeeds targetWheelSpeeds = kDriveKinematics.toWheelSpeeds(relativeSpeeds);
    targetWheelSpeeds.desaturate(edu.wpi.first.units.Units.FeetPerSecond.of(13));

    SmartDashboard.putNumber("DriveSubsystem/X Target Speeds mps", relativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveSubsystem/FL Setpoint", targetWheelSpeeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("DriveSubsystem/FR Setpoint", targetWheelSpeeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("DriveSubsystem/RL Setpoint", targetWheelSpeeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("DriveSubsystem/RR Setpoint", targetWheelSpeeds.rearRightMetersPerSecond);

    double flFF = motorFeedforward.calculate(targetWheelSpeeds.frontLeftMetersPerSecond);
    double frFF = motorFeedforward.calculate(targetWheelSpeeds.frontRightMetersPerSecond);
    double rlFF = motorFeedforward.calculate(targetWheelSpeeds.rearLeftMetersPerSecond);
    double rrFF = motorFeedforward.calculate(targetWheelSpeeds.rearRightMetersPerSecond);

    flMotor.getClosedLoopController().setReference(targetWheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, flFF, ArbFFUnits.kVoltage);
    frMotor.getClosedLoopController().setReference(targetWheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, frFF, ArbFFUnits.kVoltage);
    rlMotor.getClosedLoopController().setReference(targetWheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, rlFF, ArbFFUnits.kVoltage);
    rrMotor.getClosedLoopController().setReference(targetWheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, rrFF, ArbFFUnits.kVoltage);
  }

  // --- Private Member Functions ------------------------------------------------

  private MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        flMotor.getEncoder().getPosition(),
        frMotor.getEncoder().getPosition(),
        rlMotor.getEncoder().getPosition(),
        rrMotor.getEncoder().getPosition());
  }

  private MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        flMotor.getEncoder().getVelocity(),
        frMotor.getEncoder().getVelocity(),
        rlMotor.getEncoder().getVelocity(),
        rrMotor.getEncoder().getVelocity());
  }

  private static double getBestTargetArea(PoseEstimate estimate) {
    if (estimate.rawFiducials.length == 0) {
      return 0;
    }
    double max = estimate.rawFiducials[0].ta;
    for (var f : estimate.rawFiducials) {
      if (f.ta > max) {
        max = f.ta;
      }
    }
    return max;
  }

  /**
   * Returns true if the given estimate should be accepted and added to the
   * poseEstimator.
   */
  private boolean shouldAcceptVisionMeasurement(PoseEstimate estimate) {
    // turning faster than 720 degrees per second
    if (Math.abs(gyro.getRate()) > 720) {
      return false;
    }
    // no visible tag
    if (estimate.tagCount == 0) {
      return false;
    }
    return true;
  }

  // --- SubsystemBase -----------------------------------------------------------

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation2d(), getWheelPositions());

    // update vision
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.kCameraName,
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kCameraName);
    // strategy taken from:
    // https://www.chiefdelphi.com/t/limelight-odometry-question/433311/6
    if (shouldAcceptVisionMeasurement(estimate)) {
      double poseDifference = poseEstimator.getEstimatedPosition()
          .getTranslation()
          .getDistance(estimate.pose.getTranslation());
      double xyStdDevs = .7;
      double degStdDevs = 9999999;
      // multiple visible tags
      if (estimate.tagCount >= 2) {
        xyStdDevs = .5;
        degStdDevs = 6;
      }
      // target has large area and estimated pose is close
      else if (getBestTargetArea(estimate) > 0.8 && poseDifference < 0.5) {
        xyStdDevs = 1;
        degStdDevs = 12;
      }
      // target is further away, but estimated pose is closer
      else if (getBestTargetArea(estimate) > 0.1 && poseDifference < 0.3) {
        xyStdDevs = 2;
        degStdDevs = 30;
      }
      poseEstimator.addVisionMeasurement(
          estimate.pose, estimate.timestampSeconds,
          VecBuilder.fill(xyStdDevs, xyStdDevs, Units.degreesToRadians(degStdDevs)));
    }

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    posePublisher.set(poseEstimator.getEstimatedPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Yaw", gyro::getYaw, null);
    builder.addDoubleProperty("FL Measurement", flMotor.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("FR Measurement", frMotor.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("RL Measurement", rlMotor.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("RR Measurement", rrMotor.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("X Speed", () -> kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond,
        null);
    builder.addDoubleProperty("Y Speed", () -> kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).vyMetersPerSecond,
        null);
  }
}
