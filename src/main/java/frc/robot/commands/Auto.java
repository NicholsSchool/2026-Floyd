package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

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

    public Command shootFirstPosition(){
        BooleanSupplier autoAimReady = () -> (shooter.isAtGoal() && turret.isAtGoal() && redirector.isAtGoal());
        return new SequentialCommandGroup(AutoAim().until(autoAimReady), new InstantCommand(() -> indexer.indexWithVoltage())
        .withTimeout(AutoConstants.INITIAL_AUTO_SHOOT_TIME));
    }

}
