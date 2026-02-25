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
  private static final RobotType robot = RobotType.ROBOT_REAL_FRANKENLEW;
  public static final boolean DRIVE_ROBOT_RELATIVE =
      false; // set to true to override all field relative and instead command in robot-relative.

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static final boolean TUNING_MODE = true;
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final double METERS_PER_INCH = 0.0254;
  public static final double KG_PER_LB = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.08;

  public static boolean DISABLE_HAL = false;

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
      public static int REDUX = 20;

      public static final int FRONT_LEFT_DRIVE = 21;
      public static final int BACK_LEFT_DRIVE = 22;
      public static final int FRONT_RIGHT_DRIVE = 24;
      public static final int BACK_RIGHT_DRIVE = 23;
  
      public static final int FRONT_LEFT_PIVOT = 25;
      public static final int BACK_LEFT_PIVOT = 26;
      public static final int FRONT_RIGHT_PIVOT = 28;
      public static final int BACK_RIGHT_PIVOT = 27;

      public static final int FRONT_LEFT_ENCODER = 29;
      public static final int BACK_LEFT_ENCODER = 30;
      public static final int FRONT_RIGHT_ENCODER = 32;
      public static final int BACK_RIGHT_ENCODER = 31;

      public static int kMaxFrontLeftDrivingCanId = 24;
      public static int kMaxFrontRightDrivingCanId = 26;
      public static int kMaxRearLeftDrivingCanId = 22;
      public static int kMaxRearRightDrivingCanId = 28;
      
      public static int kMaxFrontLeftTurningCanId = 23;
      public static int kMaxFrontRightTurningCanId = 25;
      public static int kMaxRearLeftTurningCanId = 21;
      public static int kMaxRearRightTurningCanId = 27;

    public static final int SHOOTER = 43;

    public static final int REDIRECTOR = 41;

    public static final int REDIRECTOR_ENCODER = 40;

    public static final int INDEXER = 42;

    public static final int INTAKE_WHEEL = 0; //TODO: set correct CAN ID
    public static final int INTAKE_PIVOT = 0; //TODO: set correct CAN ID

    public static final int CANDLE = 50;

    public static final int TURRET = 0; //TODO: set correct CAN ID
    public static final int TURRET_ENCODER = 0; //TODO: set correct CAN ID

  }

  public static final class RobotConstants {
   
  }

}
