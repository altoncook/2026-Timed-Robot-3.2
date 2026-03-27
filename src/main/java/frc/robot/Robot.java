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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import static frc.robot.Constants.LEDConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default: Backup from Hub and then Launch";
  private static final String kLaunchFromEitherSide = "Launch from either Side";
  private static final String kLaunchRight = "Launch from Right side and go to Feeder";
  private static final String kLaunchLeft = "Launch from Left side and go to Floor bin";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String kControllerTank = "Default: Controller set to tank controls";
  private static final String kControllerArcade = "Controller set to Arcade controls";
  private String m_controllerSelected;
  private final SendableChooser<String> m_controllerChooser = new SendableChooser<>();

  private final SparkMax leftForwardDriveLead = new SparkMax(LEFT_FORWARD_ID, MotorType.kBrushed);
  private final SparkMax leftBackDriveFollower = new SparkMax(LEFT_BACK_ID, MotorType.kBrushed);
  private final SparkMax rightForwardDriveLead = new SparkMax(RIGHT_FORWARD_ID, MotorType.kBrushed);
  private final SparkMax rightBackDriveFollower = new SparkMax(RIGHT_BACK_ID, MotorType.kBrushed);
  private final SparkMax leftIntakeShootExpel = new SparkMax(LEFT_LAUNCH_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rightBinIntakeExpel = new SparkMax(RIGHT_LAUNCH_MOTOR_ID, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftForwardDriveLead, rightForwardDriveLead);

  private double m_driveScale;
  private double m_rotateScale;

  // slew rate limiters stop a value from increasing more then the value set
  private SlewRateLimiter tankLeftFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
  private SlewRateLimiter tankRightFilter = new SlewRateLimiter(TANK_DRIVE_CONTROLLER_DAMPING);
  private SlewRateLimiter arcadeFilter = new SlewRateLimiter(ARCADE_DRIVE_CONTROLLER_DAMPING);

  private final Timer autoTimer = new Timer();
  private final Timer spinUpTimer = new Timer();

  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  private final XboxController opController = new XboxController(OPERATOR_CONTROLLER_PORT);

  // the LED strip that we use has a grb setup, so flip green and red values
  private final LEDPattern red = LEDPattern.solid(Color.kGreen)
      .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); // Used for launch default
  private final LEDPattern green = LEDPattern.solid(Color.kRed)
      .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); // Used for launch left
  private final LEDPattern blue = LEDPattern.solid(Color.kBlue)
      .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); // Used for launch right
  private final LEDPattern purple = LEDPattern.solid(Color.kCyan)
      .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); // Used for launch either side
  private final LEDPattern scrollTeleOp = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kNavy, Color.kGold)
      .scrollAtRelativeSpeed(Percent.per(Second).of(LED_SCROLL_SPEED))
      .atBrightness(Dimensionless.ofRelativeUnits(LED_BRIGHTNESS_PERCENT, Percent)); // Used for teleop, just to look cool

  private final LEDPattern redBlink = red.blink(Seconds.of(LED_BLINKING_RATE));
  private final LEDPattern greenBlink = green.blink(Seconds.of(LED_BLINKING_RATE));
  private final LEDPattern blueBlink = blue.blink(Seconds.of(LED_BLINKING_RATE));
  private final LEDPattern purpleBlink = purple.blink(Seconds.of(LED_BLINKING_RATE));

  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLED m_led;

  /*
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code
   */
  public Robot() {
    CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("Default: Backup from Hub and then Launch", kDefaultAuto);
    m_chooser.addOption("Launch from either Side", kLaunchFromEitherSide);
    m_chooser.addOption("Launch from Right side and go to Feeder", kLaunchRight);
    m_chooser.addOption("Launch from Left side and go to Floor bin", kLaunchLeft);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_controllerChooser.setDefaultOption("Default: Controller set to tank controls", kControllerTank);
    m_controllerChooser.addOption("Controller set to Arcade controls", kControllerArcade);
    SmartDashboard.putData("Controller choices", m_controllerChooser);

    m_led = new AddressableLED(LED_PWM_PORT);
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(92);
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data, will not work without it being updated
    m_led.setData(m_ledBuffer);
    m_led.start();

    // ------------------Drive Configs------------------------
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

    // --------------------Shooter Configs-------------------------------------
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(LEFT_LAUNCH_CURRENT_LIMIT);
    leftConfig.inverted(false);
    leftIntakeShootExpel.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(RIGHT_LAUNCH_CURRENT_LIMIT);
    rightConfig.inverted(false);
    rightBinIntakeExpel.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    updateSelected();

    autoTimer.start();
    autoTimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    updateLEDS();
    switch (m_autoSelected) {
      case kLaunchRight: // Routine: Launch from Right side then turn and go to Human Feeder
        if (autoTimer.get() < SPINUP_SECONDS) {// Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
        } else if (autoTimer.get() < SHOOT_SECONDS) {// Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
        } else if (autoTimer.get() < SHOOT_SECONDS + 1) {// Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.7, -.7);
        } else if (autoTimer.get() < SHOOT_SECONDS + 1.5) {// Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.4, -.4);
        } else if (autoTimer.get() < SHOOT_SECONDS + 3) {// Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.7, -.7);
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchLeft: // Routine: Launch from Right side then turn and go to Human Feeder
        if (autoTimer.get() < SPINUP_SECONDS) {// Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
        } else if (autoTimer.get() < SHOOT_SECONDS) {// Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
        } else if (autoTimer.get() < SHOOT_SECONDS + 1) {// Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.7, -.7);
        } else if (autoTimer.get() < SHOOT_SECONDS + 1.5) {// Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.4, .4);
        } else if (autoTimer.get() < SHOOT_SECONDS + 3) {// Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.7, -.7);
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchFromEitherSide: // Routine: Set robot 30 inches to side of Hub, angled to point at Hub with outside corner at start line
        if (autoTimer.get() < SPINUP_SECONDS) {// spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
        } else if (autoTimer.get() < SHOOT_SECONDS) {
          if (autoTimer.get() > 6 && autoTimer.get() < 6.4) {
            leftIntakeShootExpel.setVoltage(-LAUNCHING_LEFT_VOLTAGE);
            rightBinIntakeExpel.setVoltage(-LAUNCHING_RIGHT_VOLTAGE);
          } else {
            leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
            rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
          }
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kDefaultAuto: // This has several item tests commented out followed by Default routine starting centered on Hub
      default:

        // leftForwardDriveLead.set(.35);
        // leftBackDriveFollower.set(.35);
        // rightForwardDriveLead.set(.35);
        // rightBackDriveFollower.set(.35);
        // rightShooterFeeder.set(.7);
        // leftShooterIntakeShooter.set(.97);

        if (autoTimer.get() < SPINUP_SECONDS - 0.5) {// spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
          rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
          myDrive.tankDrive(-.5, -.5);
        } else if (autoTimer.get() < SHOOT_SECONDS) {
          if (autoTimer.get() > 6 && autoTimer.get() < 6.4) {
            leftIntakeShootExpel.setVoltage(-LAUNCHING_LEFT_VOLTAGE);
            rightBinIntakeExpel.setVoltage(-LAUNCHING_RIGHT_VOLTAGE);
          } else {
            leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
            rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
          }
          myDrive.tankDrive(.0, .0); // stop
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(0, 0);
        }
        break;
    }
  }

  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    updateSelected();

    autoTimer.stop();
    spinUpTimer.start();
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    scrollTeleOp.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
    // ---------------------------------Drive Mechanisme...Driver...............................................
    if (driverController.getLeftBumperButton()) { // slow mode
      m_driveScale = SLOW_DRIVE_SCALE;
      m_rotateScale = SLOW_ROTATION_SCALE;
    } else {
      m_driveScale = DRIVE_SCALE;
      m_rotateScale = DRIVE_SCALE;
    }

    if (m_controllerSelected == kControllerTank) { // get value that is set from smartDashBoard
      if (USE_DRIVE_DAMPING)
        myDrive.tankDrive(tankLeftFilter.calculate(driverController.getLeftY()) * m_driveScale, tankRightFilter.calculate(driverController.getRightY()) * m_driveScale);
       else 
        myDrive.tankDrive(driverController.getLeftY() * m_driveScale, driverController.getRightY() * m_driveScale);
    } else {
      if (USE_DRIVE_DAMPING)
        myDrive.arcadeDrive(arcadeFilter.calculate(driverController.getLeftY()) * m_driveScale, driverController.getRightX() * m_rotateScale);
      else 
        myDrive.arcadeDrive(driverController.getLeftY() * m_driveScale, driverController.getRightX() * m_rotateScale);
    }

    // ---------------------------------Fuel Mechanism...Fuel Operator----------------------------------------
    if (opController.getRightBumperButton()) { // press Right Bumper to launch fuel
      if (opController.getRightBumperButtonPressed()) {
        spinUpTimer.reset();
      }
      if (spinUpTimer.get() < SPINUP_SECONDS) { // spinning up the Launcher
        leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
        rightBinIntakeExpel.setVoltage(SPINUP_RIGHT_VOLTAGE);
      } else {
        leftIntakeShootExpel.setVoltage(LAUNCHING_LEFT_VOLTAGE);
        rightBinIntakeExpel.setVoltage(LAUNCHING_RIGHT_VOLTAGE);
      }
    } else if (opController.getAButton()) { // press A to Intake fuel from Floor
      leftIntakeShootExpel.setVoltage(INTAKING_LEFT_VOLTAGE);
      rightBinIntakeExpel.setVoltage(INTAKING_RIGHT_VOLTAGE);
    } else if (opController.getYButton()) { // press Y to Expel fuel
      leftIntakeShootExpel.setVoltage(-INTAKING_LEFT_VOLTAGE);
      rightBinIntakeExpel.setVoltage(-INTAKING_RIGHT_VOLTAGE);
    } else { // turn stuff off if nothing is pressed
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    updateSelected();
    updateLEDS();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

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
    return INTAKING_LEFT_VOLTAGE;
  }

  public static double getIntakingRightVoltage() {
    return INTAKING_RIGHT_VOLTAGE;
  }

  public static double getLaunchingLeftVoltage() {
    return LAUNCHING_LEFT_VOLTAGE;
  }

  public static double getLaunchingRightVoltage() {
    return LAUNCHING_RIGHT_VOLTAGE;
  }

  public static double getSpinupLeftVoltage() {
    return SPINUP_LEFT_VOLTAGE;
  }

  public static double getSpinupRightVoltage() {
    return SPINUP_RIGHT_VOLTAGE;
  }

  public static double getAutoSpinupSeconds() {
    return SPINUP_SECONDS;
  }

  public static double getAutoShootSeconds() {
    return SHOOT_SECONDS;
  }

  public void updateSelected() { // Only run when starting a mode, not during.
    m_autoSelected = m_chooser.getSelected();
    m_controllerSelected = m_controllerChooser.getSelected();
    // System.out.println("Auto mode selected: " + m_autoSelected);
    // System.out.println("Controller mode selected: " + m_controllerSelected);
  }

  private void updateLEDS() {
    if (m_controllerSelected == kControllerArcade) { // If controller is arcade, start blinking
      switch (m_autoSelected) {
        case kLaunchRight:
          greenBlink.applyTo(m_ledBuffer);
          break;
        case kLaunchLeft:
          blueBlink.applyTo(m_ledBuffer);
          break;
        case kLaunchFromEitherSide:
          purpleBlink.applyTo(m_ledBuffer);
          break;
        default:
          redBlink.applyTo(m_ledBuffer);
          break;
      }
    } else { // If controller is tank, don't blink
      switch (m_autoSelected) {
        case kLaunchRight:
          green.applyTo(m_ledBuffer);
          break;
        case kLaunchLeft:
          blue.applyTo(m_ledBuffer);
          break;
        case kLaunchFromEitherSide:
          purple.applyTo(m_ledBuffer);
          break;
        default:
          red.applyTo(m_ledBuffer);
          break;
      }
    }
    m_led.setData(m_ledBuffer);
  }
}
