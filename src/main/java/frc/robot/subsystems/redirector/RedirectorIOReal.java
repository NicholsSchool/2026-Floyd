package frc.robot.subsystems.redirector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CAN;

public class RedirectorIOReal implements RedirectorIO{

    private TalonFX redirectorMotor;

    public RedirectorIOReal(){
        redirectorMotor = new TalonFX(CAN.REDIRECTOR);

         TalonFXConfiguration redirectorConfig = new TalonFXConfiguration();
        redirectorConfig.CurrentLimits.StatorCurrentLimit = RedirectorConstants.REDIRECTOR_CURRENT_LIMIT;
        redirectorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        redirectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        redirectorMotor.getConfigurator().apply(redirectorConfig);
    }

    @Override
    public void updateInputs(RedirectorIOInputs inputs){
        inputs.appliedVolts = redirectorMotor.getMotorVoltage().getValueAsDouble();
        inputs.velocityRadPerSec = redirectorMotor.getVelocity().getValueAsDouble();
        inputs.currentAngle = redirectorMotor.getPosition().getValueAsDouble();
        inputs.currentAmps = redirectorMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        redirectorMotor.setVoltage(voltage);
    } 
    
}
