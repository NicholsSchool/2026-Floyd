package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {

    private TalonFX indexerRightMotor;
    private TalonFX indexerLeftMotor;

    public IndexerIOReal(){
        indexerLeftMotor = new TalonFX(Constants.CAN.INDEXER_LEFT, "Shooter");
        indexerRightMotor = new TalonFX(Constants.CAN.INDEXER_RIGHT, "Shooter");
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.INDEXER_CURRENT_LIMIT;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        indexerRightMotor.getConfigurator().apply(indexerConfig);
        indexerLeftMotor.getConfigurator().apply(indexerConfig);
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {        
        inputs.indexerRightVoltage = indexerRightMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerLeftVoltage = indexerLeftMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerRightSupplyVoltage = indexerRightMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerLeftSupplyVoltage = indexerLeftMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerRightCurrentAmps = indexerRightMotor.getStatorCurrent().getValueAsDouble();
        inputs.indexerLeftCurrentAmps = indexerLeftMotor.getStatorCurrent().getValueAsDouble();
    }
    
    @Override
    public void setVoltageIndexer(double voltage) {
        indexerRightMotor.setVoltage(voltage);
        indexerLeftMotor.setVoltage(-voltage);
    }
}
