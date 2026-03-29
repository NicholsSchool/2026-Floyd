package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CAN;

public class IntakeIOReal implements IntakeIO {

    private TalonFX intakeMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(CAN.INTAKE_WHEEL);
        pivotMotor = new TalonFX(CAN.INTAKE_PIVOT);
        pivotEncoder = new CANcoder(CAN.INTAKE_PIVOT_ENCODER);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PIVOT_IN_ANGLE * IntakeConstants.PIVOT_RATIO); // Always start with pivot IN

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.WHEEL_CURRENT_LIMIT;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(intakeConfig);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wheelMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.wheelMotorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();

        // getPosition is in revolutions, so convert to radians
        inputs.pivotAngleRadians = -pivotEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
        
    }


    @Override
    public void setWheelMotorVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
    
}