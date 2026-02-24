package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.Constants.CAN;

public class TurretIOReal implements TurretIO{
    private TalonFX turretMotor;
    private CANcoder turretEncoder;

    public TurretIOReal(){
        turretMotor = new TalonFX(CAN.TURRET);
        turretEncoder = new CANcoder(CAN.TURRET_ENCODER);

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.TURRET_CURRENT_LIMIT;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotor.getConfigurator().apply(turretConfig);

    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        inputs.appliedVolts = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.velocityRadPerSec = turretMotor.getVelocity().getValueAsDouble();
        inputs.currentAngle = turretEncoder.getPosition().getValueAsDouble();
        inputs.currentAmps = turretMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }
}
