package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle.CandleConstants;
import frc.robot.subsystems.Candle.Candle.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class CandleUpdate extends InstantCommand{
    Candle candle;
    Drive drive;
    Intake intake;

    public CandleUpdate(Candle candle, Drive drive, Intake intake){
        this.candle = candle;
        this.drive = drive;
        this.intake = intake;
        addRequirements(candle);
    }
    @Override
    public void execute(){
        if(Math.hypot(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy) > 0.1){
            candle.setColor(CandleConstants.WRONG_RED, Subsystem.DRIVE);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.DRIVE);
        }

        if(intake.getWheelVoltage() > 0.5){
            candle.setColor(CandleConstants.MAIZE, Subsystem.INTAKE);
        }else{
            candle.setColor(CandleConstants.FLOYD_PINK, Subsystem.INTAKE);
        }
    }
    
}
