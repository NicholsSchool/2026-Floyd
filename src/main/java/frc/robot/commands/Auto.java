package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoConfig.PickupRegion;
import frc.robot.commands.AutoConfig.ShootingRegion;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.PivotPreset;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Circle;

public class Auto {
    public Drive drive;
    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;
    public Redirector redirector;

    public Auto(Drive drive, Intake intake, Indexer indexer, Shooter shooter, Redirector redirector){
        this.drive = drive;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.redirector = redirector;
    }

    public Command splineV5ToPose(Supplier<Pose2d> pose, Supplier<Circle> circle, boolean slowmode) {
        var splToPose =
            new SplineV5ToPose(this.drive, () -> {return pose.get();}, () -> {return circle.get();});
        return splToPose.until(splToPose::atGoal);
    }

    public Command AutoAim(){
        return new ShooterAutoAim(drive, shooter);
    }

    public Command goToCenter(PickupRegion pickupLocation){
        Pose2d desiredPose;
        if(pickupLocation.equals(PickupRegion.DEPOT)){
            return new DriveToPose(drive, AllianceFlipUtil.applyRotate(new Pose2d(FieldConstants.Depot.depotCenter.toTranslation2d(), new Rotation2d(Math.PI / 2))));
        }
        if(pickupLocation.equals(PickupRegion.LEFT)){
            desiredPose = new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 7.5), new Rotation2d(0));
        }else{
            desiredPose = new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 0.8), new Rotation2d(0));
        }
        return new DriveToPose(drive, AllianceFlipUtil.applyRotate(desiredPose));
    }

    public Command intakeCenter(boolean followThrough, PickupRegion pickupRegion){
        if(followThrough){

            if(pickupRegion.equals(PickupRegion.LEFT)){
                return new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 0.6), new Rotation2d(-Math.PI / 2))));
            }else{
                return new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 7.5), new Rotation2d(Math.PI / 2))));
            }
        }else{
              if(pickupRegion.equals(PickupRegion.LEFT)){
                return new SequentialCommandGroup(new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 5.0), new Rotation2d(-Math.PI / 2)))), 
                new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 7.5), new Rotation2d(-Math.PI / 2)))));
            }else{
                return new SequentialCommandGroup(new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 3.0), new Rotation2d(Math.PI / 2)))), 
                new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(AutoConstants.CENTER_INTAKE_FILE, 0.6), new Rotation2d(Math.PI / 2)))));
            }
        }
    }

    public Command driveToShootPos(ShootingRegion shootingRegion, PickupRegion pickupRegion, boolean followThrough){
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
        if((pickupRegion.equals(PickupRegion.LEFT) && followThrough) || (pickupRegion.equals(PickupRegion.RIGHT) && !followThrough)){
        return new SequentialCommandGroup(new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(2.2, 0.6), new Rotation2d(0.0)))),
             (new DriveToPose(drive, AllianceFlipUtil.apply(shootingPos))));
        }else{
            return new SequentialCommandGroup(new DriveToPose(drive, true, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(2.2, 7.5), new Rotation2d(0.0)))),
             (new DriveToPose(drive, AllianceFlipUtil.apply(shootingPos))));
        }
    }

    public Command goToPreloadShootPosition(){
        return new DriveToPose(drive, () -> AllianceFlipUtil.applyRotate(new Pose2d(new Translation2d(3.135, 5.73), new Rotation2d(Math.toRadians(-51.3)))));
        //straight back: 2.2,4
        //human player: 0.8, 0.6
            //FLR
        //Match 7 Auto: 1.6, 5.8, 35 degrees?
        //Match 8 Auto: 1.5, 4, 75 deg
        //Match 9 Auto: 1.2, 0.7
            //TVR

        //Match 2/4 Auto: 3.135, 2.218, 51 deg
        //match something auto:  3.135 5.73 =51.3 (otherside bump left i think)
    }


    public Command auto(){
        if(AutoConfig.centerAuto){
            return new SequentialCommandGroup(goToPreloadShootPosition().withTimeout(3), AutoAim(), new InstantCommand(() -> intake.setPivotGoal(PivotPreset.MID)),
            new WaitCommand(AutoConstants.AUTO_REV_TIME), indexer.commandIndex());
        }else{
        return new SequentialCommandGroup(
            goToCenter(AutoConfig.pickupLocationOne),
             new InstantCommand(() -> intake.setPivotGoal(PivotPreset.OUT)),
              new WaitCommand(0.5), 
         new ParallelCommandGroup(intakeCenter(AutoConfig.followThroughOne, AutoConfig.pickupLocationOne),
         intake.commandIntake(), indexer.commandBackdex()),
          driveToShootPos(AutoConfig.shootingPositionOne, AutoConfig.pickupLocationOne, AutoConfig.followThroughOne),
           AutoAim(), new InstantCommand(() -> intake.setPivotGoal(PivotPreset.MID)), new WaitCommand(AutoConstants.AUTO_REV_TIME), indexer.commandIndex());
        }
    }
                                                                                                        


}
