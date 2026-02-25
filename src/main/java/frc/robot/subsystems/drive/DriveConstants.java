package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {

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
