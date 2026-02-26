package frc.robot.subsystems.drive;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public final class ModuleConstants {
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
