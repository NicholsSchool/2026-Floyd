package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Circle;

public class AutoConstants {
    public static double ANGLE_FINISH_THRESHOLD = 0.1;
    public static  double DRIVE_FINISH_THRESHOLD = 0.1;


    public static final double splineFinishThreshold = 0.4;
    public static final double splineAngleFinishThreshold = Math.PI / 12.0;
    public static final double SplineV5LinearMultiplier = 1.5;
    public static final double SplineV5CircularMultiplier = 2.0;
    public static final double dotProductThreshold = 0.05;
    public static final double splineV5Multipler = 0.7;

    public static final Circle trenchCircle = new Circle(new Translation2d(4.6,4), 3.3)
}
