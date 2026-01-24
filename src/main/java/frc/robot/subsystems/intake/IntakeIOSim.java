package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

    private static final DCMotor intakeModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim intakeMotor =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(intakeModel, 0.025, IntakeConstants.INTAKE_RATIO),
    intakeModel);

    private static final DCMotor dropperModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim dropperMotor =
    new DCMotorSim(
    LinearSystemId.createDCMotorSystem(dropperModel, 0.1, IntakeConstants.DROPPER_RATIO),
    dropperModel);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        intakeMotor.update(Constants.LOOP_PERIOD_SECS);
        dropperMotor.update(Constants.LOOP_PERIOD_SECS);

        inputs.dropperAngleRadians = dropperMotor.getAngularPositionRad();
    }

    @Override
    public void setWheelMotorVoltage(double volts) {
        intakeMotor.setInputVoltage(volts);
    }

    @Override
    public void setDropperMotorVoltage(double volts) {
        dropperMotor.setInputVoltage(volts);
    }

    @Override
    public void setWheelBrakeMode(boolean brake) {
        
    }

    @Override
    public void setDropperBrakeMode(boolean brake) {
        
    }
    
}
