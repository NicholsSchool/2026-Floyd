package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.BradyMathLib;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpointRPM = 0.0;
    private double voltageCmd = 0.0;
    PIDController pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    BangBangController bangBangController = new BangBangController();
    SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

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
        return 12 * ((( inputs.velocityRPM - setpointRPM) > ShooterConstants.bangBangToleranceRPM) ? bangBangController.calculate(inputs.velocityRPM) : 0.0);
    }

    @AutoLogOutput
    public double getRPM(){
      return inputs.velocityRPM;
    }

    public void stop() {
        setRPM(0.0);
    }
}