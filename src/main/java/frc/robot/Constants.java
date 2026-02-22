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

  public static boolean disableHAL = false;
  public static boolean tuningMode = false;

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

    public static final int SHOOTER = 42;

    public static final int REDIRECTOR = 45;

    public static final int REDIRECTOR_ENCODER = 40;

    public static final int INDEXER = 46;

    public static final int INTAKE_WHEEL = 0; //TODO: set correct CAN ID
    public static final int INTAKE_PIVOT = 0; //TODO: set correct CAN ID

    public static final int CANDLE = 50;

  }

  public static final class RobotConstants {
   
  }

  public static final class DriveConstants {

    public static final double MAX_LINEAR_SPEED = 3.2;
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(25);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.5);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double LOW_GEAR_SCALER = 0.6;
    public static final double TURNING_SCALAR = 0.55;

    public static final int navXPort = 0;
    
  }

  public static final class ModuleConstants {
    public static double DRIVING_STATIC_FF = 0.1;
    public static double DRIVING_VELOCITY_FF = 0.13;

    //These values should get you started
    //but calibrate them when the bot is done
    public static double DRIVING_P = 0.02;
    public static double DRIVING_I = 0.0;
    public static double DRIVING_D = 0.0;

    public static final double TURNING_P = 6.2;
    public static final double TURNING_I = 0.0;
    public static final double TURNING_D = 0.12;
    public static final double TURNING_FF = 0.0;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;

    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final double MODULE_ZERO_ENCODER_OFFSET = 0.0;
    public static final double MODULE_ONE_ENCODER_OFFSET = 0.0;
    public static final double MODULE_TWO_ENCODER_OFFSET = 0.0;
    public static final double MODULE_THREE_ENCODER_OFFSET = 0.0;

    public static double DRIVING_MOTOR_CURRENT_LIMIT = 30.0;
    public static final double MOTOR_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double TURNING_MOTOR_CURRENT_LIMIT = 30.0;
    public static final int kDrivingMotorCurrentLimit = 0;
    public static final int kTurningMotorCurrentLimit = 0;
  }

  public static final class ElevatorConstants{
  
  }

  public static final class IntakeConstants {

    public static final double INTAKE_VOLTAGE = 5.0;
    public static final double OUTTAKE_VOLTAGE = -5.0;

    public static final double PIVOT_P = 1.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;

    public static final double PIVOT_MAX_VEL = 3.0;
    public static final double PIVOT_MAX_ACCEL = 3.0;

    public static final double PIVOT_IN_ANGLE = 0.0;
    public static final double PIVOT_OUT_ANGLE = 1.75;

    public static final double PIVOT_RATIO = 3;
    public static final double INTAKE_RATIO = 1;

    public static final double WHEEL_CURRENT_LIMIT = 20.0;
    public static final double PIVOT_CURRENT_LIMIT = 50.0;

  }

  public static final class IndexerConstants{
    //TODO change all constant values later
    public static final double Indexer_RPM = 0.0;
    public static final double Reverse_RPM = -1.0;
    public static final double P = 0.0001;
    public static final double D = 0.0001;
    public static final double gearRatio = 1.0;
    public static final double inchesPerRadian = 1.0;
  }

  public static final class ArmConstants{

  }

//   public static final class VisionConstants {
//   // AprilTag layout
//   public static AprilTagFieldLayout APRILTAG_LAYOUT =
//       AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

//   // Camera names, must match names configured on coprocessor
//   public static String CAMERA_ZERO_NAME = "Arducam_OV2311_USB_Camera-R";
//   public static String CAMERA_ONE_NAME = "Arducam_OV2311_USB_Camera-B";

//   // Robot to camera transforms
//   // (Not used by Limelight, configure in web UI instead)
//   public static Transform3d robotToCamera0 = new Transform3d();
//   public static Transform3d robotToCamera1 = new Transform3d();

//   // Basic filtering thresholds
//   public static double maxAmbiguity = 0.3;
//   public static double maxZError = 0.75;

//   // Standard deviation baselines, for 1 meter distance and 1 tag
//   // (Adjusted automatically based on distance and # of tags)
//   public static double linearStdDevBaseline = 0.03; // Meters
//   public static double angularStdDevBaseline = 0.06; // Radians

//   // Standard deviation multipliers for each camera
//   // (Adjust to trust some cameras more than others)
//   public static double[] cameraStdDevFactors =
//       new double[] {
//         1.0, // Camera 0
//         1.0 // Camera 1 
//       };

//   // Multipliers to apply for MegaTag 2 observations
//   public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
//   public static double angularStdDevMegatag2Factor =
//       Double.POSITIVE_INFINITY; // No rotation data available

//       public static final int initVisionCountTreshold = 100;
//       public static final double visionDistanceUpdateThreshold = 1.0; //meters
  
//       public static final double tranlationPhotonStdDevs = 0.01;
//       public static final double rotationPhotonStdDevs = 0.005;
  
//       public static final int visionStatsNumBuffer = 100;
// }


}
