package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpointRPM = 0.0;
    private double pidCmd = 0.0;
    PIDController pidController = new PIDController(ShooterConstants.VELOCITY_P, ShooterConstants.VELOCITY_I, ShooterConstants.VELOCITY_D);
    BangBangController bangBangController = new BangBangController();

    private static final LoggedTunableNumber shooterKp =
        new LoggedTunableNumber("shooter/Kp");
    private static final LoggedTunableNumber shooterKi =
        new LoggedTunableNumber("shooter/Ki");
    private static final LoggedTunableNumber shooterKd =
        new LoggedTunableNumber("shooter/Kd");
    private static final LoggedTunableNumber shooterBangBangMult =
        new LoggedTunableNumber("shooter/BangBangMult");

    public Shooter (ShooterIO io){
        this.io = io;

        shooterKp.initDefault(ShooterConstants.VELOCITY_P);
        shooterKi.initDefault(ShooterConstants.VELOCITY_I);
        shooterKd.initDefault(ShooterConstants.VELOCITY_D);
        shooterBangBangMult.initDefault(ShooterConstants.BANG_BANG_MULT);

        pidController.setP(shooterKp.get());
        pidController.setI(shooterKi.get());
        pidController.setD(shooterKd.get());
    }
    
    public void periodic(){
        if(DriverStation.isDisabled()){
            stop();
            setRPM(0.0);
        }

        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        updateTunables();

        pidCmd += 0.1 * pidController.calculate(inputs.velocityRPM, setpointRPM);

        io.setVoltage(pidCmd
        + getBangBang()
         );
    }

    private void updateTunables() {
        if (shooterKp.hasChanged(hashCode())
            || shooterKi.hasChanged(hashCode())
            || shooterKd.hasChanged(hashCode())) {
            pidController.setP(shooterKp.get());
            pidController.setI(shooterKi.get());
            pidController.setD(shooterKd.get());
        }
    }
    
    public void setRPM(double setpoint){
        setpointRPM = setpoint;
        pidController.reset();
        bangBangController.setSetpoint(setpoint);
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
        return shooterBangBangMult.get() * bangBangController.calculate(inputs.velocityRPM);
    }

    @AutoLogOutput
    public double getRPM(){
      return inputs.velocityRPM;
    }

    public void stop() {
        setRPM(0.0);
    }
}