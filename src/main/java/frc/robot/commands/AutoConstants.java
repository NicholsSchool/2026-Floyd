package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Circle;

public class AutoConstants {
    public static double ANGLE_FINISH_THRESHOLD = 0.1;
    public static  double DRIVE_FINISH_THRESHOLD = 0.1;
    
    public static final double SPLINE_FINISH_THRESHOLD = 0.4;
    public static final double SPLINE_ANGLE_FINISH_THRESHOLD = 0.1;
    public static final double SPLINE_V5_LINEAR_MULTIPLIER = 1.5;
    public static final double SPLINE_V5_CIRCULAR_MULTIPLIER = 2.0;
    public static final double DOT_PRODUCT_THRESHOLD = 0.05;
    public static final double SPLINE_V5_MULTIPLIER = 0.7;

    public static final double CENTER_INTAKE_FILE = 7.5;

    public static final Circle TRENCH_CIRCLE = new Circle(new Translation2d(4.6,4), 3.25);

    public static final double INITIAL_AUTO_SHOOT_TIME = 2.0;

    public static final Pose2d RIGHT_SHOOT_POS = new Pose2d(new Translation2d(3.0, 1.0), new Rotation2d(Math.toRadians(90.0)));

    public static final Pose2d CENTER_SHOOT_POS = new Pose2d(new Translation2d(3.0, 4.0), new Rotation2d(Math.toRadians(0.0)));

    public static final Pose2d LEFT_SHOOT_POS = new Pose2d(new Translation2d(3.0, 7.0), new Rotation2d(Math.toRadians(Math.toRadians(-90.0))));

    public static final double INTAKE_TIME = 10.0;

    public static final double AUTOAIM_GASLIGHT = 0.5;

    public static final double SHOOT_ON_MOVE_FUTURE_MULTIPLIER = 0.3;

    public static final double AUTO_REV_TIME = 3.0;
}
