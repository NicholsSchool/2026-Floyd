package frc.robot.subsystems.intake;
 
public final class IntakeConstants {

    public static final double INTAKE_VOLTAGE = 10.0;
    public static final double INDEX_VOLTAGE = 7.0;
    public static final double OUTTAKE_VOLTAGE = -5.0;
    public static final double BACKDEX_VOLTAGE = -7.0;

    public static final double PIVOT_P = 10.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;

    public static final double PIVOT_MAX_VEL = 3.0;
    public static final double PIVOT_MAX_ACCEL = 3.0;

    //0.125 was the original in angle
    public static final double PIVOT_IN_ANGLE = 0.2;
    public static final double PIVOT_OUT_ANGLE = 1.7;
    //1.5 is where it stalls on top of a ball
    public static final double PIVOT_MID_ANGLE = 0.9;

    public static final double PIVOT_RATIO = 90.0;
    public static final double INTAKE_RATIO = 1;

    public static final double WHEEL_CURRENT_LIMIT = 39.0;
    public static final double PIVOT_CURRENT_LIMIT = 35.0;

    public static final double PIVOT_FEED_VOLTAGE = -10;

  }