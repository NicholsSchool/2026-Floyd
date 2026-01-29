package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class ShooterIOSim implements ShooterIO {

    private static final DCMotor outtakeMotorModel = DCMotor.getKrakenX60(1);

     private final DCMotorSim outtakeMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(outtakeMotorModel, 0.0035, 1.0),
          outtakeMotorModel);

    public void updateInputs(ShooterIOInputs inputs){
        outtakeMotor.update(0.02);
        inputs.supplyVoltage = outtakeMotor.getInputVoltage();
        inputs.currentAmps = outtakeMotor.getCurrentDrawAmps();
        inputs.velocityRPM = outtakeMotor.getAngularVelocityRPM();
    }

    public void setVoltage(double voltage) {
        outtakeMotor.setInputVoltage(voltage);
    }

    public void stop(){
        outtakeMotor.setInputVoltage(0.0);
    }
}