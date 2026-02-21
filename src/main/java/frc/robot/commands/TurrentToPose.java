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
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;


public class TurrentToPose extends Command {
    private final Turret turret;
    private final Supplier<Double> angleSupplier;
    private double targetAngle;

    private boolean running = false;

    private static final LoggedTunableNumber angleOffset = new LoggedTunableNumber("TurrentToPose/AngleOffset");
    private static final LoggedTunableNumber aimDeadband = new LoggedTunableNumber("TurrentToPose/AimDeadband");

    private static final LoggedTunableNumber turretKp = new LoggedTunableNumber("TurrentToPose/Kp");
    private static final LoggedTunableNumber turretKi = new LoggedTunableNumber("TurrentToPose/Ki");
    private static final LoggedTunableNumber turretKd = new LoggedTunableNumber("TurrentToPose/Kd");
    private static final LoggedTunableNumber turretMaxVel = new LoggedTunableNumber("TurrentToPose/MaxVelRad");
    private static final LoggedTunableNumber turretMaxAccel = new LoggedTunableNumber("TurrentToPose/MaxAccelRad");
    private static final LoggedTunableNumber turretMinAngle = new LoggedTunableNumber("TurrentToPose/MinAngle");
    private static final LoggedTunableNumber turretMaxAngle = new LoggedTunableNumber("TurrentToPose/MaxAngle");
    private static final LoggedTunableNumber onGoalTolerance = new LoggedTunableNumber("TurrentToPose/OnGoalTolerance");

    private ProfiledPIDController turretPidController;

    static {
        angleOffset.initDefault(0.0);
        aimDeadband.initDefault(Math.toRadians(1.5));
        
        turretKp.initDefault(TurretConstants.TURRET_P);
        turretKi.initDefault(TurretConstants.TURRET_I);
        turretKd.initDefault(TurretConstants.TURRET_D);
        turretMaxVel.initDefault(TurretConstants.TURRET_MAX_VEL_RAD);
        turretMaxAccel.initDefault(TurretConstants.TURRET_MAX_ACCEL_RAD);
        turretMinAngle.initDefault(TurretConstants.TURRET_MIN_ANGLE);
        turretMaxAngle.initDefault(TurretConstants.TURRET_MAX_ANGLE);
        onGoalTolerance.initDefault(0.02);
    }
    private double currentGoalAngle = Double.NaN;

    public TurrentToPose(Turret turret, double angle) {
        this(turret, () -> angle);
    }

    public TurrentToPose(Turret turret, Supplier<Double> angleSupplier) {
        this.turret = turret;
        this.angleSupplier = angleSupplier;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        running = true;
        targetAngle = angleSupplier.get();
        double desired = computeDesiredTurretAngle();
        if (turretPidController == null) {
            turretPidController = new ProfiledPIDController(
                turretKp.get(), turretKi.get(), turretKd.get(),
                new TrapezoidProfile.Constraints(turretMaxVel.get(), turretMaxAccel.get()));
        }
    turretPidController.reset(turret.getAngle());
        turretPidController.setGoal(desired);
    turret.setTargetPosition(desired);
    currentGoalAngle = desired;
        
        double voltage = turretPidController.calculate(turret.getAngle());
        turret.setVoltage(voltage);
        Logger.recordOutput("TurrentToPose/DesiredAngle", desired);
    }

    @Override
    public void execute() {
        targetAngle = angleSupplier.get();
        double desired = computeDesiredTurretAngle();
        
        if (Math.abs(wrapAngle(desired - turret.getAngle())) > aimDeadband.get()) {
            currentGoalAngle = desired;
        }
        
        if (turretPidController != null) {
            turretPidController.setPID(turretKp.get(), turretKi.get(), turretKd.get());
            turretPidController.setConstraints(new TrapezoidProfile.Constraints(turretMaxVel.get(), turretMaxAccel.get()));
            double voltage = turretPidController.calculate(turret.getAngle());
            turret.setVoltage(voltage);
        }
        Logger.recordOutput("TurrentToPose/DesiredAngle", desired);
    }

    private double computeDesiredTurretAngle() {
        double offset = angleOffset.get();
        return angleSupplier.get() + offset;
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
        
        if (Double.isNaN(currentGoalAngle)) return false;
        boolean atGoal = Math.abs(turret.getAngle() - currentGoalAngle) < onGoalTolerance.get();
        if (atGoal) running = false;
        return atGoal;
    }

    public boolean isRunning() {
        return running;
    }
}
