package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
    
    private static final DCMotor simModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim simMotor =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(simModel, 0.025, IndexerConstants.gearRatio),
    simModel);
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        simMotor.update(Constants.loopPeriodSecs);
        
        inputs.indexerMotorVoltage = simMotor.getInputVoltage();
        inputs.indexerSupplyVoltage = simMotor.getInputVoltage();
        inputs.indexerVelocityINPerSec = simMotor.getAngularVelocityRadPerSec() * IndexerConstants.inchesPerRadian;
        inputs.indexerCurrentAmps = simMotor.getCurrentDrawAmps();
        inputs.indexerPositionIN = simMotor.getAngularPositionRad() * IndexerConstants.inchesPerRadian;
    }
    
    @Override
    public void setMotorVoltage(double voltage) {
        simMotor.setInputVoltage(voltage);
    }
    
}