package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ShooterIOReal implements ShooterIO {

    private TalonFX shooterMotor;

    public ShooterIOReal(){
        shooterMotor = new TalonFX(CAN.SHOOTER,"shooter");

         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotor.getConfigurator().apply(config);

    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.currentAmps = shooterMotor.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = shooterMotor.getSupplyVoltage().getValueAsDouble();
        inputs.velocityRPM = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void setVoltage(double voltage){
        shooterMotor.setVoltage(voltage);
    }
}
