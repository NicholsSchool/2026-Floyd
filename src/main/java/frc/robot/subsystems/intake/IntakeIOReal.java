package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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

    private SparkFlex wheelMotor1;
    private SparkFlex wheelMotor2;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    public IntakeIOReal() {
        wheelMotor1 = new SparkFlex(CAN.INTAKE_WHEEL_ONE, MotorType.kBrushless);
        wheelMotor2 = new SparkFlex(CAN.INTAKE_WHEEL_TWO, MotorType.kBrushless);
        pivotMotor = new TalonFX(CAN.INTAKE_PIVOT);
        pivotEncoder = new CANcoder(CAN.INTAKE_PIVOT_ENCODER);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PIVOT_IN_ANGLE * IntakeConstants.PIVOT_RATIO); // Always start with pivot IN

        SparkFlexConfig wheelConfig = new SparkFlexConfig();
        wheelConfig.smartCurrentLimit((int) IntakeConstants.WHEEL_CURRENT_LIMIT);
        wheelConfig.idleMode(IdleMode.kBrake);
        wheelMotor1.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wheelMotor2.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wheelMotorVoltage = wheelMotor1.getAppliedOutput() * wheelMotor1.getBusVoltage();
        inputs.wheelMotorCurrent = wheelMotor1.getOutputCurrent();

        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();

        // getPosition is in revolutions, so convert to radians
        inputs.pivotAngleRadians = -pivotEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
        
    }

    @Override
    public void setWheelMotorVoltage(double volts) {
        wheelMotor1.setVoltage(volts);
        wheelMotor2.setVoltage(-volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
    
}