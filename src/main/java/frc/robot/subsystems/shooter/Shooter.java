package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpointRPM = 0.0;
    private double voltageCmd = 0.0;
    PIDController pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    BangBangController bangBangController = new BangBangController();

    public Shooter (ShooterIO io){
        this.io = io;
    }
    
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        voltageCmd += 0.1 * pidController.calculate(inputs.velocityRPM, setpointRPM);

        io.setVoltage(voltageCmd + getBangBang());

    }
    
    public void setRPM(double setpoint){
        setpointRPM = setpoint;
        pidController.reset();
        bangBangController.setSetpoint(setpoint);
    }

    @AutoLogOutput
    public double getSetpointRPM(){
        return setpointRPM;
    }

    @AutoLogOutput
    public double getVoltageCmd(){
        return voltageCmd;
    }

    @AutoLogOutput
    public double getBangBang(){
        if(( Math.abs(inputs.velocityRPM - setpointRPM)) < ShooterConstants.bangBangToleranceRPM || (setpointRPM == 0.0)){
            return 0.0;
        } 
        return ShooterConstants.bangBangMult * bangBangController.calculate(inputs.velocityRPM);
    }

    @AutoLogOutput
    public double getRPM(){
      return inputs.velocityRPM;
    }

    public void stop() {
        setRPM(0.0);
    }
}