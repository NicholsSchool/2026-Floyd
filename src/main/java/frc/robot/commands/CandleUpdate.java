package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle.CandleConstants;
import frc.robot.subsystems.Candle.Candle.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;


public class CandleUpdate extends InstantCommand{
    Candle candle;
    Drive drive;
    Intake intake;
    Turret turret;
    Redirector redirector;
    Shooter shooter;
    Indexer indexer;

    public CandleUpdate(Candle candle, Drive drive, Intake intake, Turret turret, Redirector redirector, Shooter shooter, Indexer indexer){
        this.candle = candle;
        this.drive = drive;
        this.intake = intake;
        this.turret = turret;
        this.redirector = redirector;
        this.shooter = shooter;
        this.indexer = indexer; 
        addRequirements(candle);
    }
    @Override
    public void execute(){

        if(!((turret.getAngle() > TurretConstants.TURRET_MIN_ANGLE + TurretConstants.TURRET_SOFT_LIMIT) &&
         turret.getAngle() < TurretConstants.TURRET_MAX_ANGLE - TurretConstants.TURRET_SOFT_LIMIT)){
            candle.setColor(CandleConstants.WRONG_RED, Subsystem.TURRET);
        }else if(turret.isAtGoal()){
            candle.setColor(CandleConstants.BEST_GREEN, Subsystem.TURRET);
        }else if(turret.getTurretMode() == Turret.TurretMode.MANUAL){
            candle.setColor(CandleConstants.IVORY, Subsystem.TURRET);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.TURRET);
        }

        if(redirector.isAtGoal()){
            candle.setColor(CandleConstants.BEST_GREEN, Subsystem.REDIRECTOR);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.REDIRECTOR);
        }

        if(shooter.isAtGoal()){
            candle.setColor(CandleConstants.BEST_GREEN, Subsystem.SHOOTER);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.SHOOTER);
        }

        if(indexer.hasBall()){
            candle.setColor(CandleConstants.BEST_GREEN, Subsystem.INDEXER);
        }else if(indexer.getVoltageCommand() > 1.0){
            candle.setColor(CandleConstants.IVORY, Subsystem.INDEXER);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.INDEXER);
        }
    
            
    }
    
}
