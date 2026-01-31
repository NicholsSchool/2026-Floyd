package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

    private static final DCMotor wheelModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim wheelMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(wheelModel, 0.025, IntakeConstants.INTAKE_RATIO),
        wheelModel);

    private static final DCMotor pivotModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim pivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(pivotModel, 0.1, IntakeConstants.PIVOT_RATIO),
        pivotModel);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        wheelMotor.update(Constants.LOOP_PERIOD_SECS);
        pivotMotor.update(Constants.LOOP_PERIOD_SECS);

        inputs.pivotAngleRadians = pivotMotor.getAngularPositionRad();

        inputs.pivotMotorVoltage = pivotMotor.getInputVoltage();
        inputs.pivotMotorCurrent = pivotMotor.getCurrentDrawAmps();

        inputs.wheelMotorVoltage = wheelMotor.getInputVoltage();
        inputs.wheelMotorCurrent = wheelMotor.getCurrentDrawAmps();

    }

    @Override
    public void setWheelMotorVoltage(double volts) {
        wheelMotor.setInputVoltage(volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        pivotMotor.setInputVoltage(volts);
    }
    
}
