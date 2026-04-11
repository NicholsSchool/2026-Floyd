package frc.robot.subsystems.redirector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CAN;

public class RedirectorIOReal implements RedirectorIO{

    private TalonFX redirectorMotor;
    private CANcoder redirectorEncoder;

    public RedirectorIOReal(){
       
    }

    @Override
    public void updateInputs(RedirectorIOInputs inputs){
        
    }

    @Override
    public void setVoltage(double voltage){
       
    } 
    
}
