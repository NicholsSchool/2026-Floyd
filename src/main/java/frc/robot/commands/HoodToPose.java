// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Supplier<Double> angleSupplier;
  private double targetAngle;

  private boolean running = false;

  private static final LoggedTunableNumber redirectorKp = new LoggedTunableNumber("HoodToPose/Kp");
  private static final LoggedTunableNumber redirectorKi = new LoggedTunableNumber("HoodToPose/Ki");
  private static final LoggedTunableNumber redirectorKd = new LoggedTunableNumber("HoodToPose/Kd");
  private static final LoggedTunableNumber redirectorMaxVel = new LoggedTunableNumber("HoodToPose/MaxVelRad");
  private static final LoggedTunableNumber redirectorMaxAccel = new LoggedTunableNumber("HoodToPose/MaxAccelRad");
  private static final LoggedTunableNumber redirectorMinAngle = new LoggedTunableNumber("HoodToPose/MinAngle");
  private static final LoggedTunableNumber redirectorMaxAngle = new LoggedTunableNumber("HoodToPose/MaxAngle");
  private static final LoggedTunableNumber redirectorOnGoalTolerance = new LoggedTunableNumber("HoodToPose/OnGoalTolerance");
  private double currentGoalAngle = Double.NaN;

  private ProfiledPIDController redirectorPidController;

  static {
    redirectorKp.initDefault(RedirectorConstants.REDIRECTOR_P);
    redirectorKi.initDefault(RedirectorConstants.REDIRECTOR_I);
    redirectorKd.initDefault(RedirectorConstants.REDIRECTOR_D);
    redirectorMaxVel.initDefault(RedirectorConstants.REDIRECTOR_MAX_VEL_RAD);
    redirectorMaxAccel.initDefault(RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD);
  redirectorMinAngle.initDefault(RedirectorConstants.REDIRECTOR_MIN_ANGLE);
  redirectorMaxAngle.initDefault(RedirectorConstants.REDIRECTOR_MAX_ANGLE);
  redirectorOnGoalTolerance.initDefault(0.02);
  }

  public HoodToPose(Redirector redirector, double angle) {
    this(redirector, () -> angle);
  }

  public HoodToPose(Redirector redirector, Supplier<Double> angleSupplier) {
    this.redirector = redirector;
    this.angleSupplier = angleSupplier;
    addRequirements(redirector);
  }

  @Override
  public void initialize() {
    running = true;
    targetAngle = angleSupplier.get();
    double desired = computeDesiredAngle();
    if (redirectorPidController == null) {
      redirectorPidController =
          new ProfiledPIDController(
              redirectorKp.get(), redirectorKi.get(), redirectorKd.get(),
              new TrapezoidProfile.Constraints(redirectorMaxVel.get(), redirectorMaxAccel.get()));
    }
    redirectorPidController.reset(redirector.getAngle());
  redirectorPidController.setGoal(desired);
  redirector.setTargetPosition(desired);
  currentGoalAngle = desired;
    double voltage = redirectorPidController.calculate(redirector.getAngle());
    redirector.setVoltage(voltage);
    Logger.recordOutput("HoodToPose/DesiredAngle", desired);
  }

  @Override
  public void execute() {
    targetAngle = angleSupplier.get();
    double desired = computeDesiredAngle();
    redirector.setTargetPosition(desired);
    if (redirectorPidController != null) {
      redirectorPidController.setGoal(desired);
      currentGoalAngle = desired;
    }
    if (redirectorPidController != null) {
      redirectorPidController.setPID(redirectorKp.get(), redirectorKi.get(), redirectorKd.get());
      redirectorPidController.setConstraints(
          new TrapezoidProfile.Constraints(redirectorMaxVel.get(), redirectorMaxAccel.get()));
      double voltage = redirectorPidController.calculate(redirector.getAngle());
      redirector.setVoltage(voltage);
    }
    Logger.recordOutput("HoodToPose/DesiredAngle", desired);
  }

  private double computeDesiredAngle() {
    return targetAngle;
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  @Override
  public boolean isFinished() {
    if (Double.isNaN(currentGoalAngle)) return false;
    boolean atGoal = Math.abs(redirector.getAngle() - currentGoalAngle) < redirectorOnGoalTolerance.get();
    if (atGoal) running = false;
    return atGoal;
  }
  public boolean isRunning() {
    return running;
  }
}