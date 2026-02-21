package frc.robot.subsystems.redirector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;


public class Redirector extends SubsystemBase
{
    private final RedirectorIOInputsAutoLogged inputs = new RedirectorIOInputsAutoLogged();
    private RedirectorIO io;


    private double accelRad = 0.0;

  private double targetAngle = 0.0;
  private double voltageCmd = 0.0;
    private boolean reachedTargetPosition = true;
    private double voltageCmdManual = 0.0;

    private enum RedirectorMode{
      MANUAL,
      GO_TO_POSITION
    }

    private RedirectorMode redirectorMode;

    public Redirector(RedirectorIO io){
        this.io = io;
        reachedTargetPosition = true;
        this.redirectorMode = RedirectorMode.GO_TO_POSITION;
        setTargetPosition(getAngle());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Redirector", inputs);

        switch (redirectorMode) {
          case GO_TO_POSITION:
            voltageCmd = 0.0;
            voltageCmdManual = 0.0;
            break;
          case MANUAL:
            voltageCmd = 0.0;
            setTargetPosition(getAngle());
            break;
        }

        io.setVoltage(voltageCmdManual + voltageCmd);
    }


    public void setTargetPosition(double targetAngle) {
        this.targetAngle = targetAngle;
        reachedTargetPosition = false;
      }

    public Command runGoToPositionCommand(double targetAngle){
    redirectorMode = RedirectorMode.GO_TO_POSITION;
    if ((targetAngle > RedirectorConstants.REDIRECTOR_MAX_ANGLE || targetAngle < RedirectorConstants.REDIRECTOR_MIN_ANGLE)) {
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetAngle );
    return new InstantCommand(() -> setTargetPosition(targetAngle), this);
  }

       public void runManualPosition(double stickPosition){
        if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
          redirectorMode = RedirectorMode.MANUAL;
          voltageCmdManual = stickPosition * RedirectorConstants.REDIRECTOR_MANUAL_SCALAR;
        }
        else{
          redirectorMode = RedirectorMode.GO_TO_POSITION;
          }
        
      } 
        
  @AutoLogOutput
  public RedirectorMode getRedirectorMode(){
    return redirectorMode;
  }

  @AutoLogOutput
  public double getAngle(){
    return inputs.currentAngle;
  }

  @AutoLogOutput
  public double targetAngle(){
    return targetAngle;
  }

  @AutoLogOutput
  public double getAcceleration() {
    return accelRad;
  }

  @AutoLogOutput
  public double getAppliedVoltage(){
    return inputs.appliedVolts;
  }

  @AutoLogOutput
  public double getVoltageCommandPid() {
    return voltageCmd;
  }

  public void setVoltage(double volts) {
    this.voltageCmd = volts;
  }

  @AutoLogOutput
  public double getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
  
    return Math.abs(getAngle() - targetAngle) < 0.02;
  }
  
}