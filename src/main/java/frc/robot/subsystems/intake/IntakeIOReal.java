package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CAN;

public class IntakeIOReal implements IntakeIO {

    private SparkFlex wheelMotor;
    private TalonFX pivotMotor;

    public IntakeIOReal() {
        wheelMotor = new SparkFlex(CAN.INTAKE_WHEEL, MotorType.kBrushless);
        pivotMotor = new TalonFX(CAN.INTAKE_PIVOT);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PIVOT_IN_ANGLE * IntakeConstants.PIVOT_RATIO); // Always start with pivot IN

        SparkFlexConfig wheelConfig = new SparkFlexConfig();
        wheelConfig.smartCurrentLimit((int) IntakeConstants.WHEEL_CURRENT_LIMIT);
        wheelConfig.idleMode(IdleMode.kBrake);
        wheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wheelMotorVoltage = wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage();
        inputs.wheelMotorCurrent = wheelMotor.getOutputCurrent();

        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();

        // getPosition is in revolutions, so convert to radians and account for gear ratio.
        inputs.pivotAngleRadians = pivotMotor.getPosition().getValueAsDouble() * (2.0 * Math.PI) / IntakeConstants.PIVOT_RATIO;
        
    }

    @Override
    public void setWheelMotorVoltage(double volts) {
        wheelMotor.setVoltage(volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
    
}