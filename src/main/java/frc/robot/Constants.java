package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_REAL;
  public static final boolean DRIVE_ROBOT_RELATIVE =
      false; // set to true to override all field relative and instead command in robot-relative.

  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final double METERS_PER_INCH = 0.0254;
  public static final double KG_PER_LB = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.08;

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static boolean DISABLE_HAL = false;
  public static boolean TUNING_MODE = true;

  public static RobotType getRobot() {
    return RobotBase.isReal() ? robot : RobotType.ROBOT_SIM;
  }

  public static enum RobotType {
    ROBOT_REAL_FRANKENLEW, // a real robot (LEW ZEALAND)
    ROBOT_REAL, // a real robot (JANICE)
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM, // simulation
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {
    //frankenlew

      public static int kMaxFrontLeftDrivingCanId = 50;
      public static int kMaxFrontRightDrivingCanId = 56;
      public static int kMaxRearLeftDrivingCanId = 54;
      public static int kMaxRearRightDrivingCanId = 52;
      
      public static int kMaxFrontLeftTurningCanId = 51;
      public static int kMaxFrontRightTurningCanId = 57;
      public static int kMaxRearLeftTurningCanId = 55;
      public static int kMaxRearRightTurningCanId = 53;

      //Floyd

      public static int REDUX = 20;
      public static final int FRONT_LEFT_DRIVE = 50;
      public static final int BACK_LEFT_DRIVE = 54;
      public static final int FRONT_RIGHT_DRIVE = 56;
      public static final int BACK_RIGHT_DRIVE = 52;
  
      public static final int FRONT_LEFT_PIVOT = 51;
      public static final int BACK_LEFT_PIVOT = 55;
      public static final int FRONT_RIGHT_PIVOT = 57;
      public static final int BACK_RIGHT_PIVOT = 53;

      public static final int FRONT_LEFT_ENCODER = 61;
      public static final int BACK_LEFT_ENCODER = 59;
      public static final int FRONT_RIGHT_ENCODER = 58;
      public static final int BACK_RIGHT_ENCODER = 60;

    public static final int SHOOTER_BOTTOM = 30;
    public static final int SHOOTER_TOP = 31;

    public static final int REDIRECTOR = 32;

    public static final int REDIRECTOR_ENCODER = 33;

    public static final int INDEXER = 40;
    public static final int FEEDER = 39;

    public static final int INTAKE_WHEEL_ONE = 62;
    public static final int INTAKE_WHEEL_TWO = 61;
    public static final int INTAKE_PIVOT = 41; 
    public static final int INTAKE_PIVOT_ENCODER = 42;

    public static final int CANDLE = 21;

    public static final int TURRET = 34; 
    public static final int TURRET_ENCODER = 35;

  }

  public static final class RobotConstants {
   
  }

}
