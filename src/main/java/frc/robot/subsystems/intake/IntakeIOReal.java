package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class IntakeIOReal implements IntakeIO {

    private TalonFX intakeMotorTop;
    private TalonFX intakeMotorBottom;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    public IntakeIOReal() {
        intakeMotorTop = new TalonFX(Constants.CAN.INTAKE_TOP, "Shooter");
        intakeMotorBottom = new TalonFX(Constants.CAN.INTAKE_BOTTOM, "Shooter");
        pivotMotor = new TalonFX(Constants.CAN.INTAKE_PIVOT, "Shooter");
        pivotEncoder = new CANcoder(Constants.CAN.INTAKE_PIVOT_ENCODER, "Shooter");

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PIVOT_IN_ANGLE * IntakeConstants.PIVOT_RATIO);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.WHEEL_CURRENT_LIMIT;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotorTop.getConfigurator().apply(intakeConfig);
        intakeMotorBottom.getConfigurator().apply(intakeConfig);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorTopVoltage = intakeMotorTop.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorTopCurrent = intakeMotorTop.getStatorCurrent().getValueAsDouble();

        inputs.intakeMotorBottomVoltage = intakeMotorBottom.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorBottomCurrent = intakeMotorBottom.getStatorCurrent().getValueAsDouble();

        inputs.pivotAngleRadians = (pivotEncoder.getAbsolutePosition().getValueAsDouble() * -2.43912) + 0.797594;
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        
    }


    @Override
    public void setWheelMotorVoltage(double volts) {
        intakeMotorTop.setVoltage(0);
        intakeMotorBottom.setVoltage(-volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        pivotMotor.setVoltage(-volts);
    }
    
}