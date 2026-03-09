// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.management.MemoryType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default: Backup from Hub and then Launch";
  private static final String kLaunchFromEitherSide = "Launch from either Side";
  private static final String kLaunchRight = "Launch from Right side and go to Feeder";
  private static final String kLaunchLeft = "Launch from Left side and go to Floor bin";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax leftForwardDriveLead = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax leftBackDriveFollower = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax rightForwardDriveLead = new SparkMax(6, MotorType.kBrushed);
  private final SparkMax rightBackDriveFollower = new SparkMax(7, MotorType.kBrushed);
  private final SparkMax rightBinIntakeExpel = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax leftIntakeShootExpel = new SparkMax(2, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftForwardDriveLead,rightForwardDriveLead);

  private final Timer autoTimer = new Timer();
  private final Timer spinUpTimer = new Timer();

  private boolean isTankMode = true;

  private final XboxController driverController = new XboxController(0);
  private final XboxController opController = new XboxController(1);

  //--------------------------Intake/Feeder/Launcher Parameters--------------------------------------
  private static final double INTAKING_Left_Voltage = - 9;
  private static final double INTAKING_Right_Voltage = - 12;
  private static final double LAUNCHING_Left_Voltage = - 10.6;
  private static final double LAUNCHING_Right_Voltage = 9; 
  private static final double SPINUP_Left_Voltage = 1;
  private static final double SPINUP_Right_Voltage = -6;
  private static final double SPINUP_Seconds = 1;
  private static final double SHOOT_Seconds = 9; //this is includes the SPINUP time for an actual (SHOOT minus SPINUP) seconds
  
  //-----------------------drive speed parameters--------------------------------------------------------
  private double driveSpeed = 1;
  private double rotateSpeed = 1;

  /**
   * This function is run when the rBKJHKJobot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("Default: Backup from Hub and then Launch", kDefaultAuto);
    m_chooser.addOption("Launch from either Side", kLaunchFromEitherSide);
 //   m_chooser.addOption("Launch from Right side and go to Feeder", kLaunchRight);
  //          ]]\ =
      m_chooser.addOption("Launch from Left side and go to Floor bin", kLaunchLeft);

    SmartDashboard.putData("Auto choices", m_chooser);

    //------------------Drive Configs------------------------
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.voltageCompensation(12);
    driveConfig.smartCurrentLimit(60);

    driveConfig.follow(leftForwardDriveLead);
    leftBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveConfig.follow(rightForwardDriveLead);
    rightBackDriveFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    driveConfig.inverted(false);
    leftForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveConfig.inverted(true);
    rightForwardDriveLead.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //--------------------Shooter Configs-------------------------------------
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(60);
    leftConfig.inverted(false);
    leftIntakeShootExpel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(60);
    rightConfig.inverted(false);
    rightBinIntakeExpel.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    autoTimer.start();
    autoTimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kLaunchRight: // Routine: Launch from Right side then turn and go to Humman Feeder
        if(autoTimer.get() < SPINUP_Seconds){//Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        }
        else if(autoTimer.get() < SHOOT_Seconds){//Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 1){//Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 1.5){//Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.4, .4);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 3){//Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);

          myDrive.tankDrive(.7, .7);
        }
        else{
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchLeft: // Routine: Launch from Right side then turn and go to Humman Feeder"
        if(autoTimer.get() < SPINUP_Seconds){//Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        }
        else if(autoTimer.get() < SHOOT_Seconds){//Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 1){//Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 1.5){//Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.4, -.4);
        }
        else if(autoTimer.get() < SHOOT_Seconds + 3){//Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);

          myDrive.tankDrive(.7, .7);
        }
        else{
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchFromEitherSide: // Routine: Set robot 30 inches to side of Hub, angled to point at Hub with outside corner at start line
        if(autoTimer.get() < SPINUP_Seconds){//spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        }
        else if(autoTimer.get() < SHOOT_Seconds){
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        }
        else{
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kDefaultAuto: // This has several item tests commented out followed by Default routine starting centered on Hub
      default:
      
        //leftForwardDriveLead.set(.35);
        //leftBackDriveFollower.set(.35);
        //rightForwardDriveLead.set(.35);
        //rightBackDriveFollower.set(.35);
        //rightShooterFeeder.set(.7);
        //leftShooterIntakeShooter.set(.97);
       
        if(autoTimer.get() < SPINUP_Seconds - 0.5){//spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
          myDrive.tankDrive(-.4,-.4);
        }
          else if(autoTimer.get() < SHOOT_Seconds){
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
          myDrive.tankDrive(.0,.0);
        }
          else{
          leftIntakeShootExpel.setVoltage(12);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(0, 0);
        }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoTimer.stop();
    spinUpTimer.start();
  }
    
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  //---------------------------------Drive Mechanisme...Driver...............................................
    if(driverController.getLeftBumperButton()){  // slow mode
      driveSpeed = .8;
      rotateSpeed = .7;
    }
    else if(driverController.getRightBumperButton()){ // fast mode
      driveSpeed = 1;
      rotateSpeed = 1;
    }

    if (driverController.getAButtonPressed()) {
      isTankMode = !isTankMode;
    }

    if (isTankMode) {
          myDrive.tankDrive(driverController.getLeftY(), driverController.getRightY());
    } else {
      myDrive.arcadeDrive(driverController.getLeftY(), driverController.getLeftX() * rotateSpeed);
    }


  //---------------------------------Fuel Mechanism...Fuel Operator----------------------------------------
    if(opController.getRightBumperButton()){ // press Right Bumper to launch fuel
      if(opController.getRightBumperButtonPressed()){
        spinUpTimer.reset();
      }
      if(spinUpTimer.get() < SPINUP_Seconds){ // spinning up the Launcher
        leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
        rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
      }
      else{
      leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
      rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
      }
    }
    else if(opController.getLeftBumperButton()){ // press A toggle to Intake fuel from Floor
      leftIntakeShootExpel.setVoltage(INTAKING_Left_Voltage);
      rightBinIntakeExpel.setVoltage(INTAKING_Right_Voltage);
    }
    else if(opController.getYButton()){ // press Y toggle to Expel fuel
      leftIntakeShootExpel.setVoltage(-INTAKING_Left_Voltage);
      rightBinIntakeExpel.setVoltage(-INTAKING_Right_Voltage);
    }
    else{ // turn stuff off
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
    }
  } 
    
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static String getKdefaultauto() {
    return kDefaultAuto;
  }

  public static String getKlaunchfromeitherside() {
    return kLaunchFromEitherSide;
  }

  public static String getKlaunchrightload() {
    return kLaunchRight;
  }

  public String getM_autoSelected() {
    return m_autoSelected;
  }

  public void setM_autoSelected(String m_autoSelected) {
    this.m_autoSelected = m_autoSelected;
  }

  public SendableChooser<String> getM_chooser() {
    return m_chooser;
  }

  public SparkMax getLeftForwardDriveLead() {
    return leftForwardDriveLead;
  }

  public SparkMax getLeftBackDriveFollower() {
    return leftBackDriveFollower;
  }

  public SparkMax getRightForwardDriveLead() {
    return rightForwardDriveLead;
  }

  public SparkMax getRightBackDriveFollower() {
    return rightBackDriveFollower;
  }

  public SparkMax getRightShooterFeeder() {
    return rightBinIntakeExpel;
  }

  public SparkMax getLeftShooterIntakeShooter() {
    return leftIntakeShootExpel;
  }

  public DifferentialDrive getMyDrive() {
    return myDrive;
  }

  public Timer getAutoTimer() {
    return autoTimer;
  }

  public XboxController getDrivController() {
    return driverController;
  }

  public XboxController getOpController() {
    return opController;
  }

  public static double getIntakingLeftVoltage() {
    return INTAKING_Left_Voltage;
  }

  public static double getIntakingRightVoltage() {
    return INTAKING_Right_Voltage;
  }

  public static double getLaunchingLeftVoltage() {
    return LAUNCHING_Left_Voltage;
  }

  public static double getLaunchingRightVoltage() {
    return LAUNCHING_Right_Voltage;
  }

  public static double getSpinupLeftVoltage() {
    return SPINUP_Left_Voltage;
  }

  public static double getSpinupRightVoltage() {
    return SPINUP_Right_Voltage;
  }

  public static double getAutoSpinupSeconds() {
    return SPINUP_Seconds;
  }

  public static double getAutoShootSeconds() {
    return SHOOT_Seconds;
  }
}
