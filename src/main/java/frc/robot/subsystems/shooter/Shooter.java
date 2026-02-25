package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpointRPM = 0.0;
    private double pidCmd = 0.0;
    PIDController pidController = new PIDController(ShooterConstants.VELOCITY_P, ShooterConstants.VELOCITY_I, ShooterConstants.VELOCITY_D);
    BangBangController bangBangController = new BangBangController();

    public Shooter (ShooterIO io){
        this.io = io;
    }
    
    public void periodic(){
        if(DriverStation.isDisabled()){
            stop();
            setRPM(0.0);
        }

        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        pidCmd += 0.1 * pidController.calculate(inputs.velocityRPM, setpointRPM);

        io.setVoltage(pidCmd
        + getBangBang()
         );
    }
    
    public void setRPM(double setpoint){
        setpointRPM = setpoint;
        pidController.reset();
        bangBangController.setSetpoint(setpoint);
    }

    public void setVelMPS(double velocity){
        //we need to find this regression
        double rpm = velocity;
        setRPM(rpm);
    }

    @AutoLogOutput
    public boolean isAtGoal(){
       return Math.abs(setpointRPM - getRPM()) < ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }

    @AutoLogOutput
    public double getSetpointRPM(){
        return setpointRPM;
    }

    @AutoLogOutput
    public double getPidCmd(){
        return pidCmd;
    }

    @AutoLogOutput
    public double getBangBang(){
        if(( Math.abs(inputs.velocityRPM - setpointRPM)) < ShooterConstants.BANG_BANG_TOLERANCE_RPM || (setpointRPM == 0.0)){
            return 0.0;
        } 
        return ShooterConstants.BANG_BANG_MULT * bangBangController.calculate(inputs.velocityRPM);
    }

    @AutoLogOutput
    public double getRPM(){
      return inputs.velocityRPM;
    }

    public void stop() {
        setRPM(0.0);
    }
}