package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoConfig.PickupRegion;
import frc.robot.commands.AutoConfig.ShootingRegion;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.PivotPreset;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Circle;

public class Auto {
    public Drive drive;
    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;
    public Turret turret;
    public Redirector redirector;

    public Auto(Drive drive, Intake intake, Indexer indexer, Shooter shooter, Turret turret, Redirector redirector){
        this.drive = drive;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.redirector = redirector;
    }

    public Command splineV5ToPose(Supplier<Pose2d> pose, Supplier<Circle> circle, boolean slowmode) {
        var splToPose =
            new SplineV5ToPose(this.drive, () -> {return pose.get();}, () -> {return circle.get();});
        return splToPose.until(splToPose::atGoal);
    }

    public Command AutoAim(){
        return new ParallelCommandGroup(new ShooterAutoAim(drive, shooter),
         new TurretAutoAim(drive, turret),
          new RedirectorAutoAim(drive, redirector));
    }

    public Command goToCenter(PickupRegion pickupLocation){
        Pose2d desiredPose;
        if(pickupLocation.equals(PickupRegion.DEPOT)){
            return new DriveToPose(drive, AllianceFlipUtil.applyRotate(new Pose2d(FieldConstants.Depot.depotCenter.toTranslation2d(), new Rotation2d(Math.PI / 2))));
        }
        if(pickupLocation.equals(PickupRegion.LEFT)){
            desiredPose = new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 7.5), new Rotation2d(-Math.PI / 2));
        }else{
            desiredPose = new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 0.8), new Rotation2d(Math.PI / 2));
        }
        return new DriveToPose(drive, AllianceFlipUtil.applyRotate(desiredPose));
    }

    public Command intakeCenter(boolean followThrough, PickupRegion pickupRegion){
        if(followThrough){
            if(pickupRegion.equals(PickupRegion.LEFT)){
                return new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 1.4), new Rotation2d(-Math.PI / 2))));
            }else{
                return new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 6.9), new Rotation2d(Math.PI / 2))));
            }
        }else{
              if(pickupRegion.equals(PickupRegion.LEFT)){
                return new SequentialCommandGroup(new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 5.0), new Rotation2d(-Math.PI / 2)))), 
                new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 7.0), new Rotation2d(Math.PI / 2)))));
            }else{
                return new SequentialCommandGroup(new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 3.0), new Rotation2d(Math.PI / 2)))), 
                new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 1.2), new Rotation2d(-Math.PI / 2)))));
            }
        }
    }

    public Command driveToShootPos(ShootingRegion shootingRegion){
        Pose2d shootingPos;
        switch (shootingRegion){
            case LEFT:
            shootingPos = AutoConstants.LEFT_SHOOT_POS;
            break;
            case CENTER:
            shootingPos = AutoConstants.CENTER_SHOOT_POS;
            break;
            case RIGHT:
            shootingPos = AutoConstants.RIGHT_SHOOT_POS;
            break;
            default:
            shootingPos = new Pose2d();

        }
        return splineV5ToPose(() -> AllianceFlipUtil.applyRotate(shootingPos), () -> AllianceFlipUtil.apply(AutoConstants.TRENCH_CIRCLE), false);
    }


    public Command auto(){
        return new SequentialCommandGroup(goToCenter(AutoConfig.pickupLocationOne), new InstantCommand(() -> intake.setPivotGoal(PivotPreset.OUT)),
         new ParallelCommandGroup(intakeCenter(AutoConfig.followThroughOne, AutoConfig.pickupLocationOne),
          new InstantCommand(() -> intake.intake()).repeatedly().withTimeout(AutoConstants.INTAKE_TIME)), new InstantCommand(() -> intake.stopWheels()),
           new ParallelCommandGroup(driveToShootPos(AutoConfig.shootingPositionOne), AutoAim()));
    }


}
