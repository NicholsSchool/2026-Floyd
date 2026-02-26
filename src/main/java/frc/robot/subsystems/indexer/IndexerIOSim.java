package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO {
    
    private static final DCMotor simModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim simMotor =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(simModel, 0.025, IndexerConstants.GEAR_RATIO),
    simModel);
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        simMotor.update(Constants.LOOP_PERIOD_SECS);
        
        inputs.indexerVoltage = simMotor.getInputVoltage();
        inputs.indexerSupplyVoltage = simMotor.getInputVoltage();
        inputs.indexerCurrentAmps = simMotor.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(double voltage) {
        simMotor.setInputVoltage(voltage);
    }
    
}