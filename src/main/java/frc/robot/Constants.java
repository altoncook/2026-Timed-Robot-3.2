package frc.robot;

/* All constants should go into this class for ease of changing */
public final class Constants {
     public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_FORWARD_ID = 5;
    public static final int LEFT_BACK_ID = 4;
    public static final int RIGHT_FORWARD_ID = 6;
    public static final int RIGHT_BACK_ID = 7;

    // Current limit for drivetrain motors. 60A is a 
    // reasonable maximum to reduce likelihood of 
    // tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }
  public static final class FuelConstants {
    // Motor controller IDs for fuel mechanism motors
    public static final int LEFT_LAUNCH_MOTOR_ID = 2;
    public static final int RIGHT_LAUNCH_MOTOR_ID = 3;

    // Current limit and nominal voltage for fuel mechanism motors
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltages for fuel stages
    public static final double SPINUP_LEFT_VOLTAGE = 1;
    public static final double SPINUP_RIGHT_VOLTAGE = -6;
    public static final double INTAKING_LEFT_VOLTAGE = -9;
    public static final double INTAKING_RIGHT_VOLTAGE = -12;
    public static final double LAUNCHING_LEFT_VOLTAGE = -10.6;
    public static final double LAUNCHING_RIGHT_VOLTAGE = 9;
    
    public static final double SPINUP_SECONDS = 1;
    public static final double SHOOT_SECONDS = 11; // this is includes the SPINUP time for an actual (SHOOT minus SPINUP) seconds
  }

  public static final class OperatorConstants {
    // Ports in driver station that each controller should go to
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Sets scale value for controller movement, only arcade uses rotation scale
    public static final double SLOW_DRIVE_SCALE = .5;
    public static final double SLOW_ROTATION_SCALE = .6;
    public static final double DRIVE_SCALE = .78;
    public static final double ROTATION_SCALE = .7;
  }

  public static final class LEDConstants {
    public static final double LED_BLINKING_RATE = 1; // second blinking rate for drive mode in disabled
    public static final int LED_BRIGHTNESS_PERCENT = 70; // can go up to 200%, might trip fuse through
    public static final int LED_SCROLL_SPEED = 50; // in percent per second, so scrolls completely once every 2 seconds
    public static final int LED_PWM_PORT = 0; 
  }
}

