package frc.robot.subsystems.turret;
//TODO: find real values for each constant

import edu.wpi.first.math.util.Units;

public class TurretConstants {
    
    public static final double TURRET_MAX_ACCEL_RAD = 100.0;
    public static final double TURRET_MAX_VEL_RAD = 200.0;
    public static final double TURRET_P = 0.01; // TODO: tune all 3 of these PID constants
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0;
    public static final double TURRET_MAX_ANGLE = Math.toRadians(110.0);
    public static final double TURRET_MIN_ANGLE = Math.toRadians(-90.0);
    public static final double TURRET_GEAR_RATIO = 1.0;
    public static final double TURRET_MANUAL_SCALAR = 1.0;
    public static final double TURRET_SOFT_LIMIT = Math.toRadians(5.0);
    public static final double TURRET_CURRENT_LIMIT = 0.0; //TODO: change this


    public static final double TURRET_OFFSET_X = Units.inchesToMeters(10);
    public static final double TURRET_OFFSET_Y = Units.inchesToMeters(-5);


}
