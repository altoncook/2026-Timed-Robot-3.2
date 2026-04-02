// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  private final XboxController opController = new XboxController(OPERATOR_CONTROLLER_PORT);
  private final Drivetrain m_drive = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);


  private Trajectory m_trajectory;

  // The Ramsete Controller to follow the trajectory.
  private final RamseteController m_ramseteController = new RamseteController();

  // The timer to use during the autonomous period.
  private Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  private Field2d m_field;



  public Robot() {
    CameraServer.startAutomaticCapture();
       // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  @Override
  public void autonomousInit() {
    // Initialize the timer.
    m_timer = new Timer();
    m_timer.start();

    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    // Update odometry.
    m_drive.updateOdometry();

    // Update robot position on Field2d.
    m_field.setRobotPose(m_drive.getPose());

    if (m_timer.get() < m_trajectory.getTotalTimeSeconds()) {
      // Get the desired pose from the trajectory.
      var desiredPose = m_trajectory.sample(m_timer.get());

      // Get the reference chassis speeds from the Ramsete controller.
      var refChassisSpeeds = m_ramseteController.calculate(m_drive.getPose(), desiredPose);

      // Set the linear and angular speeds.
      m_drive.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      m_drive.drive(0, 0);
    }
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed = -m_speedLimiter.calculate(driverController.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot = -m_rotLimiter.calculate(driverController.getRightX()) * Drivetrain.kMaxAngularSpeed;

    m_drive.drive(xSpeed, rot);
  }
}