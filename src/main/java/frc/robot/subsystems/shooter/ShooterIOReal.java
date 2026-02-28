package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ShooterIOReal implements ShooterIO {

    private TalonFX shooterMotorBottom;
    private TalonFX shooterMotorTop;

    public ShooterIOReal(){
        shooterMotorBottom = new TalonFX(CAN.SHOOTER_BOTTOM,"shooter");
        shooterMotorTop = new TalonFX(CAN.SHOOTER_TOP, "shooter");

         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorBottom.getConfigurator().apply(config);
        shooterMotorTop.getConfigurator().apply(config);

    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.currentAmps = shooterMotorBottom.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = shooterMotorBottom.getSupplyVoltage().getValueAsDouble();
        inputs.velocityRPM = -shooterMotorBottom.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void setVoltage(double voltage){
        // motors reverse of each other
        shooterMotorBottom.setVoltage(-voltage);
        shooterMotorTop.setVoltage(voltage);
    }
}
