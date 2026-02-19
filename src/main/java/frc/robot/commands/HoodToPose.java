// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.redirector.RedirectorConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that computes and sets a redirector (hood) angle based on the robot pose and a target
 * pose. Provides LoggedTunableNumbers so mapping parameters are tunable at runtime.
 */
public class HoodToPose extends Command {
  private final Redirector redirector;
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;

  private boolean running = false;

  // Mapping tunables: linear interpolation from minDistance->maxDistance to angleMin->angleMax
  private static final LoggedTunableNumber angleAtMinDistance =
      new LoggedTunableNumber("HoodToPose/AngleAtMinDistance");
  private static final LoggedTunableNumber angleAtMaxDistance =
      new LoggedTunableNumber("HoodToPose/AngleAtMaxDistance");
  private static final LoggedTunableNumber minDistance = new LoggedTunableNumber("HoodToPose/MinDistance");
  private static final LoggedTunableNumber maxDistance = new LoggedTunableNumber("HoodToPose/MaxDistance");
  private static final LoggedTunableNumber ffMinRadius = new LoggedTunableNumber("HoodToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius = new LoggedTunableNumber("HoodToPose/FFMaxRadius");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_REAL_FRANKENLEW:
      case ROBOT_REAL:
      case ROBOT_REPLAY:
      case ROBOT_SIM:
        // Defaults: distances in meters, angles in radians
        minDistance.initDefault(Units.inchesToMeters(12.0)); // 1 foot
        maxDistance.initDefault(Units.inchesToMeters(240.0)); // 20 feet
        angleAtMinDistance.initDefault(0.2); // ~11 deg
        angleAtMaxDistance.initDefault(1.0); // ~57 deg
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
      default:
        break;
    }
  }

  /** Create a HoodToPose command. */
  public HoodToPose(Redirector redirector, Drive drive, Pose2d pose) {
    this(redirector, drive, () -> pose);
  }

  /** Create a HoodToPose command. */
  public HoodToPose(Redirector redirector, Drive drive, Supplier<Pose2d> poseSupplier) {
    this.redirector = redirector;
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(redirector);
  }

  @Override
  public void initialize() {
    running = true;
    targetPose = poseSupplier.get();
    // compute and set desired redirector angle immediately
    double desired = computeDesiredAngle();
    desired = clampToRedirectorLimits(desired);
    redirector.setTargetPosition(desired);
    Logger.recordOutput("HoodToPose/DesiredAngle", desired);
  }

  @Override
  public void execute() {
    // recompute mapping each loop (tunable numbers may change)
    targetPose = poseSupplier.get();
    double desired = computeDesiredAngle();
    desired = clampToRedirectorLimits(desired);
    redirector.setTargetPosition(desired);
    Logger.recordOutput("HoodToPose/DesiredAngle", desired);
  }

  private double computeDesiredAngle() {
    // Distance from robot to target
    double distance = drive.getPose().getTranslation().getDistance(targetPose.getTranslation());
    // optional feedforward scaler (same idea as DriveToPose)
    double ffScaler =
        MathUtil.clamp(
            (distance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);

    double minD = minDistance.get();
    double maxD = maxDistance.get();
    double scalar = 0.0;
    if (maxD > minD) scalar = MathUtil.clamp((distance - minD) / (maxD - minD), 0.0, 1.0);

    double angleMin = angleAtMinDistance.get();
    double angleMax = angleAtMaxDistance.get();
    // linear interpolation between angleMin and angleMax
    double mapped = angleMin * (1.0 - scalar) + angleMax * scalar;
    // incorporate ffScaler lightly (keeps mapping behavior similar to DriveToPose style)
    return mapped * (0.75 + 0.25 * ffScaler);
  }

  private double clampToRedirectorLimits(double angle) {
    // Respect redirector limits defined in constants if they make sense.
    // Assuming RedirectorConstants limits are in degrees; if they're in radians adjust accordingly.
    double min = RedirectorConstants.REDIRECTOR_MIN_ANGLE;
    double max = RedirectorConstants.REDIRECTOR_MAX_ANGLE;
    // If constants look like degrees (values > 2*PI), convert to radians
    if (Math.abs(min) > 2 * Math.PI || Math.abs(max) > 2 * Math.PI) {
      min = Math.toRadians(min);
      max = Math.toRadians(max);
    }
    return MathUtil.clamp(angle, min, max);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    // nothing special to stop; Redirector subsystem will hold position via its controller
  }

  @Override
  public boolean isFinished() {
    // Consider finished when redirector subsystem reports it is at goal
    boolean atGoal = redirector.isAtGoal();
    if (atGoal) running = false;
    return atGoal;
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}