package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ShooterIOReal implements ShooterIO {

    private TalonFX shooterMotorBottomLeft;
    private TalonFX shooterMotorTopLeft;
    private TalonFX shooterMotorBottomRight;
    private TalonFX shooterMotorTopRight;

    public ShooterIOReal(){
        shooterMotorBottomLeft = new TalonFX(CAN.LEFT_SHOOTER_BOTTOM, "Shooter");
        shooterMotorTopLeft = new TalonFX(CAN.LEFT_SHOOTER_TOP,"Shooter");
        shooterMotorBottomRight = new TalonFX(CAN.RIGHT_SHOOTER_BOTTOM, "Shooter");
        shooterMotorTopRight = new TalonFX(CAN.RIGHT_SHOOTER_TOP,"Shooter");

         var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotorBottomLeft.getConfigurator().apply(config);
        shooterMotorTopLeft.getConfigurator().apply(config);
        shooterMotorBottomRight.getConfigurator().apply(config);
        shooterMotorTopRight.getConfigurator().apply(config);

    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.currentAmps = shooterMotorBottomLeft.getStatorCurrent().getValueAsDouble();
        inputs.supplyVoltage = shooterMotorBottomLeft.getSupplyVoltage().getValueAsDouble();
        inputs.velocityRPM = -shooterMotorBottomLeft.getVelocity().getValueAsDouble() * 60.0;
        
        inputs.bottomLeftShooterVoltage = shooterMotorBottomLeft.getMotorVoltage().getValueAsDouble();
        inputs.bottomRightShooterVoltage = shooterMotorBottomRight.getMotorVoltage().getValueAsDouble();
        inputs.topLeftShooterVoltage = shooterMotorTopLeft.getMotorVoltage().getValueAsDouble();
        inputs.topRightShooterVoltage = shooterMotorTopRight.getMotorVoltage().getValueAsDouble();

        inputs.bottomLeftShooterCurrent = shooterMotorBottomLeft.getStatorCurrent().getValueAsDouble();
        inputs.bottomRightShooterCurrent = shooterMotorBottomRight.getStatorCurrent().getValueAsDouble();
        inputs.topLeftShooterCurrent = shooterMotorTopLeft.getStatorCurrent().getValueAsDouble();
        inputs.topRightShooterCurrent = shooterMotorTopRight.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        // motors reverse of each other
        shooterMotorBottomLeft.setVoltage(-voltage);
        shooterMotorTopLeft.setVoltage(voltage);
        shooterMotorBottomRight.setVoltage(voltage);
        shooterMotorTopRight.setVoltage(-voltage);
    }
}
