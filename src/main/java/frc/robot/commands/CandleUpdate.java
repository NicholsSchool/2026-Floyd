package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle.CandleConstants;
import frc.robot.subsystems.Candle.Candle.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;


public class CandleUpdate extends InstantCommand{
    Candle candle;
    Drive drive;
    Intake intake;
    Redirector redirector;
    Shooter shooter;
    Indexer indexer;

    public CandleUpdate(Candle candle, Drive drive, Intake intake, Redirector redirector, Shooter shooter, Indexer indexer){
        this.candle = candle;
        this.drive = drive;
        this.intake = intake;
        this.redirector = redirector;
        this.shooter = shooter;
        this.indexer = indexer; 
        addRequirements(candle);
    }
    @Override
    public void execute(){

        if(intake.getPivotGoal() == IntakeConstants.PIVOT_OUT_ANGLE && (intake.getPivotAngle() - intake.getPivotGoal() > 0.1)){
            candle.setColor(CandleConstants.WRONG_RED, Subsystem.DRIVE);
        }else if(indexer.getIndexerCurrent() > 39.5){
            candle.setColor(CandleConstants.MAIZE, Subsystem.DRIVE);
        }else if(!shooter.isAtGoal() && shooter.getPidCmd() < -0.5){
            candle.setColor(CandleConstants.PERFECT_PURPLE, Subsystem.DRIVE);
        }else if(shooter.isAtGoal() && shooter.getSetpointRPM() != 0){
            candle.setColor(CandleConstants.BEST_GREEN, Subsystem.DRIVE);
        }else if(Math.abs(drive.getHubDistance() - AutoConstants.SHOOTING_DISTANCE) < AutoConstants.HUB_DISTANCE_TOLERANCE){
            candle.setColor(CandleConstants.IVORY, Subsystem.DRIVE);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.DRIVE);
        }
    
            
    }
    
}
