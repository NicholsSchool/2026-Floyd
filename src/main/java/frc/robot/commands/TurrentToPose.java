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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to point the turret at a target pose. Makes mapping parameters tunable via LoggedTunableNumber.
 */
public class TurrentToPose extends Command {
    private final Turret turret;
    private final Drive drive;
    private final Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;

    private boolean running = false;

    private static final LoggedTunableNumber angleOffset = new LoggedTunableNumber("TurrentToPose/AngleOffset");
    private static final LoggedTunableNumber aimDeadband = new LoggedTunableNumber("TurrentToPose/AimDeadband");
    private static final LoggedTunableNumber ffMinRadius = new LoggedTunableNumber("TurrentToPose/FFMinRadius");
    private static final LoggedTunableNumber ffMaxRadius = new LoggedTunableNumber("TurrentToPose/FFMaxRadius");

    static {
        // sensible defaults
        angleOffset.initDefault(0.0);
        aimDeadband.initDefault(Math.toRadians(1.5));
        ffMinRadius.initDefault(0.2);
        ffMaxRadius.initDefault(0.8);
    }

    public TurrentToPose(Turret turret, Drive drive, Pose2d pose) {
        this(turret, drive, () -> pose);
    }

    public TurrentToPose(Turret turret, Drive drive, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.drive = drive;
        this.poseSupplier = poseSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        running = true;
        targetPose = poseSupplier.get();
        double desired = computeDesiredTurretAngle();
        desired = clampToTurretLimits(desired);
        turret.setTargetPosition(desired);
        Logger.recordOutput("TurrentToPose/DesiredAngle", desired);
    }

    @Override
    public void execute() {
        targetPose = poseSupplier.get();
        double desired = computeDesiredTurretAngle();
        desired = clampToTurretLimits(desired);
        // only set new target if outside deadband to avoid unnecessary resets
        if (Math.abs(wrapAngle(desired - turret.getAngle())) > aimDeadband.get()) {
            turret.setTargetPosition(desired);
        }
        Logger.recordOutput("TurrentToPose/DesiredAngle", desired);
    }

    private double computeDesiredTurretAngle() {
        // turret physical position offset from robot center
        var robotPose = drive.getPose();
        var turretPose = robotPose.transformBy(GeomUtil.translationToTransform(TurretConstants.TURRET_OFFSET_X, TurretConstants.TURRET_OFFSET_Y));
        Translation2d toTarget = new Translation2d(targetPose.getTranslation().getX() - turretPose.getTranslation().getX(), targetPose.getTranslation().getY() - turretPose.getTranslation().getY());
        double absoluteAngle = toTarget.getAngle().getRadians();
        double robotYaw = robotPose.getRotation().getRadians();
        double relative = wrapAngle(absoluteAngle - robotYaw);
        // apply small tunable offset and light ff scaler similar to DriveToPose
        double distance = turretPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp((distance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()), 0.0, 1.0);
        double offset = angleOffset.get();
        return relative + offset * (0.5 + 0.5 * ffScaler);
    }

    private double clampToTurretLimits(double angle) {
        double min = TurretConstants.TURRET_MIN_ANGLE;
        double max = TurretConstants.TURRET_MAX_ANGLE;
        return MathUtil.clamp(angle, min, max);
    }

    private double wrapAngle(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
    }

    @Override
    public boolean isFinished() {
        boolean atGoal = turret.isAtGoal();
        if (atGoal) running = false;
        return atGoal;
    }

    public boolean isRunning() {
        return running;
    }
}
