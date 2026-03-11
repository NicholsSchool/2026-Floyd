package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    private TalonFX indexerMotor;
    private TalonFX feederMotor;

    public IndexerIOReal(){
        indexerMotor = new TalonFX(Constants.CAN.INDEXER, "Shooter");
        feederMotor = new TalonFX(Constants.CAN.FEEDER, "Shooter");
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.INDEXER_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexerMotor.getConfigurator().apply(indexerConfig);
        feederMotor.getConfigurator().apply(indexerConfig);
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {        
        inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerCurrentAmps = indexerMotor.getStatorCurrent().getValueAsDouble();
        inputs.feederVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
        inputs.feederSupplyVoltage = feederMotor.getSupplyVoltage().getValueAsDouble();
        inputs.feederCurrentAmps = feederMotor.getStatorCurrent().getValueAsDouble();
    }
    
    @Override
    public void setVoltageIndexer(double voltage) {
        indexerMotor.setVoltage(-voltage);
    }

    @Override
    public void setVoltageFeeder(double voltage){
        feederMotor.setVoltage(-voltage);
    }
}