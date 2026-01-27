package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO{
      private static final DCMotor simModel = DCMotor.getKrakenX60(1);

      private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(simModel, 0.025, TurretConstants.TURRET_GEAR_RATIO),
          simModel);
    
    double turretAppliedVolts = 0.0;

    @Override
    public void updateInputs(TurretIOInputs inputs){
        sim.update(Constants.LOOP_PERIOD_SECS);
        
        inputs.appliedVolts = sim.getInputVoltage();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentAngle = sim.getAngularPositionRad();
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.limitSwitch = true;
    }

    @Override
    public void setVoltage(double voltage) {
      sim.setInputVoltage(voltage);
    }

}