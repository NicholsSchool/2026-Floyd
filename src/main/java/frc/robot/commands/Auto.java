package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoConfig.PickupLocation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.PivotPreset;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

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

    public Command AutoAim(){
        return new ParallelCommandGroup(new ShooterAutoAim(drive, shooter),
         new TurretAutoAim(drive, turret),
          new RedirectorAutoAim(drive, redirector));
    }

    public Command goToCenter(PickupLocation pickupLocation){
        Pose2d desiredPose;
        if(pickupLocation.equals(PickupLocation.DEPOT)){
            return new DriveToPose(drive, AllianceFlipUtil.applyRotate(new Pose2d(FieldConstants.Depot.depotCenter.toTranslation2d(), new Rotation2d(Math.PI / 2))));
        }
        if(pickupLocation.equals(PickupLocation.LEFT)){
            desiredPose = new Pose2d(new Translation2d(7.5, 7.5), new Rotation2d(-Math.PI / 2));
        }else{
            desiredPose = new Pose2d(new Translation2d(7.5, 1.0), new Rotation2d(Math.PI / 2));
        }
        return new DriveToPose(drive, AllianceFlipUtil.applyRotate(desiredPose));
    }

    public Command intakeCenter(boolean followThrough, PickupLocation pickupLocation){
        if(followThrough){
            if(pickupLocation.equals(PickupLocation.LEFT)){
                return new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 1.4), new Rotation2d(-Math.PI / 2))));
            }else{
                return new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 6.9), new Rotation2d(Math.PI / 2))));
            }
        }else{
              if(pickupLocation.equals(PickupLocation.LEFT)){
                return new SequentialCommandGroup(new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 5.0), new Rotation2d(-Math.PI / 2)))), 
                new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 7.0), new Rotation2d(0.0)))));
            }else{
                return new SequentialCommandGroup(new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 3.0), new Rotation2d(0.0)))), 
                new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(7.5, 1.5), new Rotation2d(-Math.PI / 2)))));
            }
        }
    }

    public Command auto(){
        return new SequentialCommandGroup(goToCenter(AutoConfig.pickupLocationOne), new InstantCommand(() -> intake.setPivotGoal(PivotPreset.OUT)),
         new ParallelCommandGroup(intakeCenter(AutoConfig.followThroughOne, AutoConfig.pickupLocationOne),
          new InstantCommand(() -> intake.intake()).repeatedly().withTimeout(3.0)));
    }


}
