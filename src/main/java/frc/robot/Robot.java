// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.units.measure.Dimensionless;

// PhotonVision
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
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
  private static final String kDepotRun = "Shoot then Cross-Field Depot Run";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax leftForwardDriveLead = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax leftBackDriveFollower = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax rightForwardDriveLead = new SparkMax(6, MotorType.kBrushed);
  private final SparkMax rightBackDriveFollower = new SparkMax(7, MotorType.kBrushed);
  private final SparkMax rightBinIntakeExpel = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax leftIntakeShootExpel = new SparkMax(2, MotorType.kBrushed);

  private final DifferentialDrive myDrive = new DifferentialDrive(leftForwardDriveLead, rightForwardDriveLead);

  private final Timer autoTimer = new Timer();
  private final Timer spinUpTimer = new Timer();

  private static final double LEDBlinkingRate = 1; // two second blinking rate for arcade drive in disabled
  private static final int LEDBrightness = 70;// in percent

  private final LEDPattern red = LEDPattern.solid(Color.kGreen)
      .atBrightness(Dimensionless.ofRelativeUnits(LEDBrightness, Percent)); // green red invert, used for launch default
  private final LEDPattern green = LEDPattern.solid(Color.kRed)
      .atBrightness(Dimensionless.ofRelativeUnits(LEDBrightness, Percent)); // green red invert, used for launch left
  private final LEDPattern blue = LEDPattern.solid(Color.kBlue)
      .atBrightness(Dimensionless.ofRelativeUnits(LEDBrightness, Percent)); // green red invert, used for launch right
  private final LEDPattern purple = LEDPattern.solid(Color.kCyan)
      .atBrightness(Dimensionless.ofRelativeUnits(LEDBrightness, Percent)); // green red invert, used for launch either side
  private final LEDPattern scrollTeleOp = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kYellow)
      .scrollAtRelativeSpeed(Percent.per(Second).of(50))
      .atBrightness(Dimensionless.ofRelativeUnits(LEDBrightness, Percent)); // used for teleop

  private final LEDPattern redBlink = red.blink(Seconds.of(LEDBlinkingRate));
  private final LEDPattern greenBlink = green.blink(Seconds.of(LEDBlinkingRate));
  private final LEDPattern blueBlink = blue.blink(Seconds.of(LEDBlinkingRate));
  private final LEDPattern purpleBlink = purple.blink(Seconds.of(LEDBlinkingRate));

  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLED m_led;

  private static final String kControllerTank = "Default: Controller set to tank controls";
  private static final String kControllerArcade = "Controller set to Arcade controls";
  private String m_controllerSelected;
  private final SendableChooser<String> m_controllerChooser = new SendableChooser<>();

  private final XboxController driverController = new XboxController(0);
  private final XboxController opController = new XboxController(1);

  // --------------------------Intake/Feeder/Launcher Parameters--------------------------------------
  private static final double INTAKING_Left_Voltage = -9;
  private static final double INTAKING_Right_Voltage = -12;
  private static final double LAUNCHING_Left_Voltage = -10.6;
  private static final double LAUNCHING_Right_Voltage = 9;
  private static final double SPINUP_Left_Voltage = 1;
  private static final double SPINUP_Right_Voltage = -6;
  private static final double SPINUP_Seconds = 1;
  private static final double SHOOT_Seconds = 11; // this is includes the SPINUP time for an actual (SHOOT minus SPINUP) seconds

  // -----------------------Depot Run Auto constants---------------------------------------------------
  // Alliance is detected automatically from visible hub tags at autonomousInit.
  // Blue hub tags: 2,3,4,5,8,9,10,11  |  Red hub tags: 18,19,20,21,24,25,26,27
  // Trench tags facing neutral zone (used for yaw correction mid-traverse):
  //   Blue side: 7, 12  |  Red side: 17, 22  (the neutral-zone-facing ones)
  private static final int[] BLUE_HUB_TAGS    = {2, 3, 4, 5, 8, 9, 10, 11};
  private static final int[] RED_HUB_TAGS     = {18, 19, 20, 21, 24, 25, 26, 27};
  // Neutral-zone-facing trench tags. Blue side trench is on the left guardrail from blue's
  // perspective; red on right. These are the tags visible from center field as you approach.
  // IDs confirmed from 2026 REBUILT game manual figure 5-26.
  private static final int[] BLUE_TRENCH_TAGS = {7, 12};   // neutral-zone-facing, blue alliance trench
  private static final int[] RED_TRENCH_TAGS  = {17, 22};  // neutral-zone-facing, red alliance trench

  // *** ALL TIMES BELOW ARE PLACEHOLDERS - TUNE ON A REAL FIELD ***
  // Phase 1: Shoot (reuses SPINUP_Seconds + SHOOT_Seconds from above)
  // Phase 2: Back off hub after shoot
  private static final double DEPOT_BACK_OFF_SECONDS = 1.0;   // TUNE: time to clear the hub
  // Phase 3: Turn to align with bump gap (slight angle off center)
  // Positive turn = left, negative = right. Blue turns left, Red turns right (mirrored).
  private static final double DEPOT_TURN_SECONDS      = 0.4;   // TUNE: time to turn ~30-45 degrees
  private static final double DEPOT_TURN_SPEED        = 0.45;  // TUNE: turn power (differential)
  // Phase 4: Drive over our bump (6.5in tall - just power through it)
  private static final double DEPOT_BUMP_SECONDS      = 1.2;   // TUNE: time to fully clear the bump
  // Phase 5: Traverse neutral zone. Vision-corrects yaw when trench tag visible.
  private static final double DEPOT_TRAVERSE_SECONDS  = 3.5;   // TUNE: time to cross neutral zone
  private static final double DEPOT_TRAVERSE_SPEED    = 0.7;   // drive power during traverse
  // Phase 6: Drive into opponent depot area and push fuel back toward center
  private static final double DEPOT_PUSH_SECONDS      = 2.0;   // TUNE: time pushing into depot
  private static final double DEPOT_PUSH_SPEED        = 0.5;   // slower push so we don't overshoot

  // Auto traverse yaw correction gain (trench tag). Separate from teleop vision kP.
  // Keeps us straight while crossing - only activates when a trench tag is in frame.
  private static final double AUTO_TRAVERSE_kP_TURN   = 0.012; // TUNE if drifting left/right

  // Alliance determined at autonomousInit by checking which hub tags are visible.
  // true = blue, false = red, null = not yet detected (falls back to timed-only traverse)
  private Boolean detectedBlueAlliance = null;

  // -----------------------drive speed parameters--------------------------------------------------------
  private double driveSpeed = 1;
  private double rotateSpeed = 1;

  // -----------------------PhotonVision auto-distance (driver Y = hold to activate)------------------
  // Match this to your camera name in the PhotonVision UI at http://photonvision.local:5800
  private static final String PHOTON_CAMERA_NAME = "photonvision";

  // HUB AprilTag IDs -- UPDATE with confirmed 2026 REBUILT IDs from the field layout JSON
  private static final int[] HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};

  // How far from the tag to stop, in meters. Tune to your shot distance. (~59 in)
  private static final double HUB_TARGET_DISTANCE_METERS = 1.5;

  // MEASURE THESE ON YOUR ROBOT before competition
  private static final double CAMERA_HEIGHT_METERS = 0.50;  // camera lens height off carpet
  private static final double CAMERA_PITCH_DEGREES = 20.0;  // upward tilt from horizontal

  // Per 2026 game manual: HUB AprilTag centers are 44.25in off the carpet
  private static final double HUB_TAG_HEIGHT_METERS = 1.124;

  // P-loop gains -- tune kP_FORWARD if it overshoots/oscillates, kP_TURN if it hunts side-to-side
  private static final double VISION_kP_FORWARD = 0.5;
  private static final double VISION_kP_TURN    = 0.015;
  private static final double VISION_MAX_SPEED  = 0.6;
  private static final double VISION_DIST_TOL   = 0.05; // meters deadband around target

  private final PhotonCamera photonCamera = new PhotonCamera(PHOTON_CAMERA_NAME);
  // Cached last result that had targets, updated every loop in robotPeriodic.
  // Avoids the single-frame cold-read problem in autonomousInit.
  private org.photonvision.PhotonPipelineResult lastValidResult = null;


  /*
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    CameraServer.startAutomaticCapture();

    m_chooser.setDefaultOption("Default: Backup from Hub and then Launch", kDefaultAuto);
    m_chooser.addOption("Launch from either Side", kLaunchFromEitherSide);
    m_chooser.addOption("Launch from Right side and go to Feeder", kLaunchRight);
    m_chooser.addOption("Launch from Left side and go to Floor bin", kLaunchLeft);

    m_chooser.addOption("Shoot then Cross-Field Depot Run", kDepotRun);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_controllerChooser.setDefaultOption("Default: Controller set to tank controls", kControllerTank);
    m_controllerChooser.addOption("Controller set to Arcade controls", kControllerArcade);

    SmartDashboard.putData("Controller choices", m_controllerChooser);

    // PWM port 1
    // Must be a PWM header, not MXP or DIO, the LED strip that we use has a grb
    // setup and not a rgb setup, so flip green and red values
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 92, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(92);

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    // ------------------Drive Configs------------------------
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

    // --------------------Shooter Configs-------------------------------------
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
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Vision Mode Active", driverController.getYButton());
    // Cache the latest result every loop (runs during disabled too).
    // autonomousInit reads from this so it's never working from a cold single frame.
    var r = photonCamera.getLatestResult();
    if (r.hasTargets()) {
      lastValidResult = r;
    }
    SmartDashboard.putBoolean("Vision/HasCachedResult", lastValidResult != null);
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
   * <p>
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

    // Detect alliance from the cached result built up during disabled.
    // Much more reliable than a cold single-frame read at the moment auto starts.
    // Falls back to null (timed-only traverse) if camera never saw anything.
    detectedBlueAlliance = null;
    if (lastValidResult != null) {
      for (PhotonTrackedTarget t : lastValidResult.getTargets()) {
        int id = t.getFiducialId();
        for (int bid : BLUE_HUB_TAGS) {
          if (id == bid) { detectedBlueAlliance = true; break; }
        }
        if (detectedBlueAlliance != null) break;
        for (int rid : RED_HUB_TAGS) {
          if (id == rid) { detectedBlueAlliance = false; break; }
        }
        if (detectedBlueAlliance != null) break;
      }
    }
    SmartDashboard.putString("Auto/Alliance",
        detectedBlueAlliance == null ? "NOT DETECTED - timed only"
        : detectedBlueAlliance ? "BLUE" : "RED");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    updateLEDS();
    switch (m_autoSelected) {
      case kLaunchRight: // Routine: Launch from Right side then turn and go to Human Feeder
        if (autoTimer.get() < SPINUP_Seconds) {// Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        } else if (autoTimer.get() < SHOOT_Seconds) {// Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        } else if (autoTimer.get() < SHOOT_Seconds + 1) {// Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        } else if (autoTimer.get() < SHOOT_Seconds + 1.5) {// Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(-.4, .4);
        } else if (autoTimer.get() < SHOOT_Seconds + 3) {// Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchLeft: // Routine: Launch from Right side then turn and go to Human Feeder"
        if (autoTimer.get() < SPINUP_Seconds) {// Spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        } else if (autoTimer.get() < SHOOT_Seconds) {// Shoot
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        } else if (autoTimer.get() < SHOOT_Seconds + 1) {// Move away from Hub
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        } else if (autoTimer.get() < SHOOT_Seconds + 1.5) {// Turn to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.4, -.4);
        } else if (autoTimer.get() < SHOOT_Seconds + 3) {// Move to Feeder
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
          myDrive.tankDrive(.7, .7);
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kLaunchFromEitherSide: // Routine: Set robot 30 inches to side of Hub, angled to point at Hub with
                                  // outside corner at start line
        if (autoTimer.get() < SPINUP_Seconds) {// spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        } else if (autoTimer.get() < SHOOT_Seconds) {
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        } else {
          leftIntakeShootExpel.setVoltage(0);
          rightBinIntakeExpel.setVoltage(0);
        }
        break;

      case kDepotRun:
        // -----------------------------------------------------------------------
        // SHOOT THEN CROSS-FIELD DEPOT RUN
        // Alliance auto-detected at autonomousInit from visible hub AprilTags.
        // If detection failed, traverse phases use timed-only drive (no yaw correction).
        //
        // Phase timing (all values MUST be tuned on a real field):
        //   0                         -> SHOOT_Seconds      : shoot (same as default auto)
        //   SHOOT_Seconds             -> +DEPOT_BACK_OFF    : reverse off hub
        //   +DEPOT_BACK_OFF           -> +DEPOT_TURN        : turn toward bump
        //   +DEPOT_TURN               -> +DEPOT_BUMP        : drive over our bump
        //   +DEPOT_BUMP               -> +DEPOT_TRAVERSE    : cross neutral zone (vision-corrected)
        //   +DEPOT_TRAVERSE           -> +DEPOT_PUSH        : push into opponent depot
        //   +DEPOT_PUSH               ->                    : stop
        // -----------------------------------------------------------------------
        runDepotAutoPhase();
        break;

      case kDefaultAuto: // This has several item tests commented out followed by Default routine starting centered on Hub
      default:

        // leftForwardDriveLead.set(.35);
        // leftBackDriveFollower.set(.35);
        // rightForwardDriveLead.set(.35);
        // rightBackDriveFollower.set(.35);
        // rightShooterFeeder.set(.7);
        // leftShooterIntakeShooter.set(.97);

        if (autoTimer.get() < SPINUP_Seconds - 0.5) {// spinup the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
          myDrive.tankDrive(-.4, -.4);
        } else if (autoTimer.get() < SHOOT_Seconds) {
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
          if (autoTimer.get() > 6 && autoTimer.get() < 6.4) {
            myDrive.tankDrive(-.7, -.7); //shake forward
          } else if (autoTimer.get() > 6.4 && autoTimer.get() < 6.8) {
            myDrive.tankDrive(.7, .7); //shake backward
          } else {
            myDrive.tankDrive(.0, .0); //stop
          }
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
    if (driverController.getYButton()) { // hold Y to auto-distance to HUB
      runVisionDriveToHub();
    } else {
      if (driverController.getLeftBumperButton()) { // slow mode
        driveSpeed = 0.5;
        rotateSpeed = 0.6;
      } else {
        driveSpeed = 0.7;
        rotateSpeed = 0.7;
      }

      if (m_controllerSelected == kControllerTank) { // get value that is set from smartDashBoard
        myDrive.tankDrive(driverController.getLeftY() * driveSpeed, driverController.getRightY() * driveSpeed);
      } else {
        myDrive.arcadeDrive(driverController.getLeftY() * driveSpeed, driverController.getRightX() * rotateSpeed);
      }
    }

    // ---------------------------------Fuel Mechanism...Fuel Operator----------------------------------------
  //if (driverController.getYButton()) { // stop them from doing things that they shouldn't
      if (opController.getRightBumperButton()) { // press Right Bumper to launch fuel
        if (opController.getRightBumperButtonPressed()) {
          spinUpTimer.reset();
        }
        if (spinUpTimer.get() < SPINUP_Seconds) { // spinning up the Launcher
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
        } else {
          leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
          rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
        }
      } else if (opController.getAButton()) { // press A toggle to Intake fuel from Floor
        leftIntakeShootExpel.setVoltage(INTAKING_Left_Voltage);
        rightBinIntakeExpel.setVoltage(INTAKING_Right_Voltage);
      } else if (opController.getYButton()) { // press Y toggle to Expel fuel
        leftIntakeShootExpel.setVoltage(-INTAKING_Left_Voltage);
        rightBinIntakeExpel.setVoltage(-INTAKING_Right_Voltage);
      } else { // turn stuff off
        leftIntakeShootExpel.setVoltage(0);
        rightBinIntakeExpel.setVoltage(0);
      }
  //} else {
   // leftIntakeShootExpel.setVoltage(0);
   // rightBinIntakeExpel.setVoltage(0);
   //
  }


  /**
   * Executes one 20ms tick of the depot run autonomous routine.
   * Called every loop from autonomousPeriodic while kDepotRun is selected.
   *
   * Phase boundaries are defined by cumulative time offsets from SHOOT_Seconds.
   * Turn direction mirrors based on detectedBlueAlliance:
   *   Blue alliance: turn LEFT to approach the left bump (from robot's perspective
   *                  facing away from our hub), then traverse rightward across field.
   *   Red alliance:  mirror — turn RIGHT.
   * If detectedBlueAlliance is null (camera saw nothing at init), the traverse
   * phase runs straight with no yaw correction. All timed values need field tuning.
   */
  private void runDepotAutoPhase() {
    double t = autoTimer.get();

    // ---- Cumulative phase end times ----
    double tShootEnd     = SHOOT_Seconds;
    double tBackOffEnd   = tShootEnd   + DEPOT_BACK_OFF_SECONDS;
    double tTurnEnd      = tBackOffEnd + DEPOT_TURN_SECONDS;
    double tBumpEnd      = tTurnEnd    + DEPOT_BUMP_SECONDS;
    double tTraverseEnd  = tBumpEnd    + DEPOT_TRAVERSE_SECONDS;
    double tPushEnd      = tTraverseEnd + DEPOT_PUSH_SECONDS;

    // Turn direction: blue turns left to angle toward the left bump (from robot's
    // perspective backing off the hub), red mirrors to the right.
    // If alliance wasn't detected at init, skip the turn entirely (drive straight
    // through — safer than turning the wrong way).
    // Blue: left motor slower than right = turns left.
    // Red:  right motor slower than left = turns right.
    double turnLeft, turnRight;
    if (detectedBlueAlliance == null) {
      turnLeft  = 1.0; // no turn — drive straight through bump
      turnRight = 1.0;
    } else if (detectedBlueAlliance) {
      turnLeft  = 1.0 - DEPOT_TURN_SPEED; // blue: turn left
      turnRight = 1.0;
    } else {
      turnLeft  = 1.0;                    // red: turn right
      turnRight = 1.0 - DEPOT_TURN_SPEED;
    }

    if (t < SPINUP_Seconds) {
      // Phase 1a: Spin up launcher (same as default auto)
      leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
      rightBinIntakeExpel.setVoltage(SPINUP_Right_Voltage);
      myDrive.tankDrive(-.4, -.4); // back slightly while spinning up

    } else if (t < tShootEnd) {
      // Phase 1b: Shoot
      leftIntakeShootExpel.setVoltage(LAUNCHING_Left_Voltage);
      rightBinIntakeExpel.setVoltage(LAUNCHING_Right_Voltage);
      // Shake logic from default auto to dislodge stuck fuel
      if (t > 6 && t < 6.4) {
        myDrive.tankDrive(-.7, -.7);
      } else if (t > 6.4 && t < 6.8) {
        myDrive.tankDrive(.7, .7);
      } else {
        myDrive.tankDrive(0, 0);
      }

    } else if (t < tBackOffEnd) {
      // Phase 2: Shooter off, back away from hub to clear it
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      myDrive.tankDrive(DEPOT_TRAVERSE_SPEED, DEPOT_TRAVERSE_SPEED);
      // TUNE: DEPOT_BACK_OFF_SECONDS — robot should just clear the hub front face

    } else if (t < tTurnEnd) {
      // Phase 3: Turn to angle toward our bump gap.
      // The hub is ~47in wide; the bump flanks it. We need a modest angle,
      // not a full 90 — just enough to aim at the bump gap.
      // TUNE: DEPOT_TURN_SECONDS and DEPOT_TURN_SPEED together set the angle.
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      myDrive.tankDrive(
          DEPOT_TRAVERSE_SPEED * turnLeft,
          DEPOT_TRAVERSE_SPEED * turnRight
      );

    } else if (t < tBumpEnd) {
      // Phase 4: Drive over our bump. 6.5in tall — just power through.
      // TUNE: DEPOT_BUMP_SECONDS — robot should fully cross and land on neutral zone side.
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      myDrive.tankDrive(DEPOT_TRAVERSE_SPEED, DEPOT_TRAVERSE_SPEED);

    } else if (t < tTraverseEnd) {
      // Phase 5: Traverse neutral zone toward opponent side.
      // Vision yaw-corrects using our trench tags (neutral-zone-facing side)
      // if they're visible. If no tag in frame, drives straight on timer.
      // TUNE: DEPOT_TRAVERSE_SECONDS — main distance-covering phase.
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      autoTraverseWithTrenchCorrection();

    } else if (t < tPushEnd) {
      // Phase 6: We're in opponent depot area. Push fuel back toward center.
      // Drive straight slowly into the depot, ball contact does the pushing.
      // TUNE: DEPOT_PUSH_SECONDS — how long we push before stopping.
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      myDrive.tankDrive(DEPOT_PUSH_SPEED, DEPOT_PUSH_SPEED);

    } else {
      // Done — stop everything
      leftIntakeShootExpel.setVoltage(0);
      rightBinIntakeExpel.setVoltage(0);
      myDrive.tankDrive(0, 0);
    }

    // Dashboard telemetry for tuning
    SmartDashboard.putNumber("Auto/DepotPhaseTime", t);
    SmartDashboard.putNumber("Auto/ShootEnd", tShootEnd);
    SmartDashboard.putNumber("Auto/BackOffEnd", tBackOffEnd);
    SmartDashboard.putNumber("Auto/TurnEnd", tTurnEnd);
    SmartDashboard.putNumber("Auto/BumpEnd", tBumpEnd);
    SmartDashboard.putNumber("Auto/TraverseEnd", tTraverseEnd);
    SmartDashboard.putNumber("Auto/PushEnd", tPushEnd);
  }

  /**
   * Called during the traverse phase of depot run auto.
   * Checks for our alliance's trench AprilTags (neutral-zone-facing).
   * If 1 or 2 are visible, applies a P-loop yaw correction to keep us straight.
   * If no tag visible, drives straight at DEPOT_TRAVERSE_SPEED — pure timed.
   *
   * The trench tags we look for are the ones on OUR side of the field,
   * facing the neutral zone — these are visible as we cross toward the opponent.
   * Blue: tags 7 and 12. Red: tags 17 and 22.
   * We only check up to 2 tags (the ones on our trench) so we don't accidentally
   * yaw-correct toward an opponent trench tag on the far side.
   */
  private void autoTraverseWithTrenchCorrection() {
    int[] trenchTags = (detectedBlueAlliance == null)
                       ? new int[0]  // no alliance detected, drive blind
                       : detectedBlueAlliance ? BLUE_TRENCH_TAGS : RED_TRENCH_TAGS;

    var result = photonCamera.getLatestResult();
    PhotonTrackedTarget bestTag = null;
    double bestAmbiguity = Double.MAX_VALUE;

    if (result.hasTargets() && trenchTags.length > 0) {
      for (PhotonTrackedTarget t : result.getTargets()) {
        int id = t.getFiducialId();
        for (int tid : trenchTags) {
          if (id == tid && t.getPoseAmbiguity() < bestAmbiguity) {
            bestAmbiguity = t.getPoseAmbiguity();
            bestTag = t;
            break;
          }
        }
      }
    }

    double turnCorrection = 0.0;
    if (bestTag != null) {
      // Yaw: positive = tag is left of center = we're drifting right = correct left
      turnCorrection = AUTO_TRAVERSE_kP_TURN * bestTag.getYaw();
      SmartDashboard.putString("Auto/TraverseVision",
          String.format("Tag %d yaw %.1f°", bestTag.getFiducialId(), bestTag.getYaw()));
    } else {
      SmartDashboard.putString("Auto/TraverseVision", "No trench tag - timed only");
    }

    // arcadeDrive: forward = positive, rotation = positive turns left
    myDrive.arcadeDrive(DEPOT_TRAVERSE_SPEED, turnCorrection);
  }

  /**
   * Called while driver holds Y button.
   * Finds the lowest-ambiguity HUB AprilTag in frame, then:
   *   - P-loops forward/back to reach HUB_TARGET_DISTANCE_METERS
   *   - P-loops rotation to center the tag horizontally
   * Operator can still shoot independently during this — driver hands off steering only.
   */
  private void runVisionDriveToHub() {
    var result = photonCamera.getLatestResult();

    if (!result.hasTargets()) {
      myDrive.tankDrive(0, 0);
      SmartDashboard.putString("Vision/Status", "No tag visible");
      return;
    }

    // Pick lowest-ambiguity tag that is a known HUB tag ID
    PhotonTrackedTarget bestTarget = null;
    double bestAmbiguity = Double.MAX_VALUE;
    for (PhotonTrackedTarget t : result.getTargets()) {
      int id = t.getFiducialId();
      for (int hubId : HUB_TAG_IDS) {
        if (id == hubId && t.getPoseAmbiguity() < bestAmbiguity) {
          bestAmbiguity = t.getPoseAmbiguity();
          bestTarget = t;
          break;
        }
      }
    }

    if (bestTarget == null) {
      myDrive.tankDrive(0, 0);
      SmartDashboard.putString("Vision/Status", "No HUB tag in frame");
      return;
    }

    // Distance via pitch geometry -- accurate without full pose estimation
    double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        HUB_TAG_HEIGHT_METERS,
        Math.toRadians(CAMERA_PITCH_DEGREES),
        Math.toRadians(bestTarget.getPitch())
    );

    // Forward P-loop: positive error = too far away = drive forward
    double rangeError = distanceMeters - HUB_TARGET_DISTANCE_METERS;
    double forwardSpeed = 0.0;
    if (Math.abs(rangeError) > VISION_DIST_TOL) {
      forwardSpeed = VISION_kP_FORWARD * rangeError;
      forwardSpeed = Math.max(-VISION_MAX_SPEED, Math.min(VISION_MAX_SPEED, forwardSpeed));
    }

    // Yaw P-loop: getYaw() positive = tag is left of center = turn left
    double turnSpeed = VISION_kP_TURN * bestTarget.getYaw();
    turnSpeed = Math.max(-VISION_MAX_SPEED, Math.min(VISION_MAX_SPEED, turnSpeed));

    myDrive.arcadeDrive(forwardSpeed, turnSpeed);

    SmartDashboard.putNumber("Vision/Distance_m", distanceMeters);
    SmartDashboard.putNumber("Vision/RangeError_m", rangeError);
    SmartDashboard.putNumber("Vision/YawError_deg", bestTarget.getYaw());
    SmartDashboard.putNumber("Vision/TagID", bestTarget.getFiducialId());
    SmartDashboard.putString("Vision/Status",
        String.format("Tag %d | %.2fm | err %.2fm", bestTarget.getFiducialId(), distanceMeters, rangeError));
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

  public void updateSelected() {
    if (m_autoSelected != m_chooser.getSelected() || m_controllerSelected != m_controllerChooser.getSelected()) { // if any mode changes
      m_autoSelected = m_chooser.getSelected();
      m_controllerSelected = m_controllerChooser.getSelected();
      System.out.println("Auto mode selected: " + m_autoSelected);
      System.out.println("Controller mode selected: " + m_controllerSelected);
    }
  }

  private void updateLEDS() {
    if (m_controllerSelected == kControllerArcade) { // if controller is arcade, start blinking
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
    } else { // if controller is tank, don't blink
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