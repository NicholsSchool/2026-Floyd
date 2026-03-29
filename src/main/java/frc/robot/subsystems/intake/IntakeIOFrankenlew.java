package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CAN;

public class IntakeIOFrankenlew implements IntakeIO {

    private SparkFlex wheelMotor;

    public IntakeIOFrankenlew() {
        wheelMotor = new SparkFlex(CAN.INTAKE_WHEEL, MotorType.kBrushless);


        SparkFlexConfig wheelConfig = new SparkFlexConfig();
        wheelConfig.smartCurrentLimit((int) IntakeConstants.WHEEL_CURRENT_LIMIT);
        wheelConfig.idleMode(IdleMode.kBrake);
        wheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wheelMotorVoltage = wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage();
        inputs.wheelMotorCurrent = wheelMotor.getOutputCurrent();

        inputs.pivotMotorVoltage = 0.0;
        inputs.pivotMotorCurrent = 0.0;
    }

    @Override
    public void setWheelMotorVoltage(double volts) {
        wheelMotor.setVoltage(volts);
    }

    @Override
    public void setPivotMotorVoltage(double volts) {
        
    }
    
}