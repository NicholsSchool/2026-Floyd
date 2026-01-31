package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class ShooterIOSim implements ShooterIO {

    private static final DCMotor shooterMotorModel = DCMotor.getKrakenX60(1);

     private final DCMotorSim shooterMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(shooterMotorModel, 0.0035, 1.0),
          shooterMotorModel);

    public void updateInputs(ShooterIOInputs inputs){
        shooterMotor.update(0.02);
        inputs.supplyVoltage = shooterMotor.getInputVoltage();
        inputs.currentAmps = shooterMotor.getCurrentDrawAmps();
        inputs.velocityRPM = shooterMotor.getAngularVelocityRPM();
    }

    public void setVoltage(double voltage) {
        shooterMotor.setInputVoltage(voltage);
    }

    public void stop(){
        shooterMotor.setInputVoltage(0.0);
    }
}