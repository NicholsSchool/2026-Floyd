package frc.robot.subsystems.indexer;


import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    
    private static final TalonFX indexerMotor = new TalonFX(Constants.CAN.INDEXER);
    
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