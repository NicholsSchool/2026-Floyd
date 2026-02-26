package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    private TalonFX indexerMotor;

    public IndexerIOReal(){
        indexerMotor = new TalonFX(Constants.CAN.INDEXER);
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.INDEXER_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotor.getConfigurator().apply(indexerConfig);
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {        
        inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerCurrentAmps = indexerMotor.getStatorCurrent().getValueAsDouble();
    }
    
    @Override
    public void setVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }
}