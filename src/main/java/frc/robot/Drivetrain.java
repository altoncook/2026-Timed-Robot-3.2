package frc.robot;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final SparkMax leftForwardDriveLead = new SparkMax(LEFT_FORWARD_ID, MotorType.kBrushed);
  private final SparkMax leftBackDriveFollower = new SparkMax(LEFT_BACK_ID, MotorType.kBrushed);
  private final SparkMax rightForwardDriveLead = new SparkMax(RIGHT_FORWARD_ID, MotorType.kBrushed);
  private final SparkMax rightBackDriveFollower = new SparkMax(RIGHT_BACK_ID, MotorType.kBrushed);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
  
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;
  private Trajectory m_trajectory;
  private final Timer m_timer = new Timer();


  // Gains aree for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.voltageCompensation(12);
    driveConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    driveConfig.follow(leftForwardDriveLead);
    leftBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveConfig.follow(rightForwardDriveLead);
    rightBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    driveConfig.inverted(false);
    leftForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveConfig.inverted(true);
    rightForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry =
      new DifferentialDriveOdometry(
      getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Sets the desired wheel speeds.
   *
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
      m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
      m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftForwardDriveLead.setVoltage(leftOutput + leftFeedforward);
    rightForwardDriveLead.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
      getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    // Negate the angle because WPILib expects CCW positive, 
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
}
}