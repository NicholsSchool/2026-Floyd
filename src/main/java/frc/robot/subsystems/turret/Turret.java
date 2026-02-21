package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;


public class Turret extends SubsystemBase
{
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private TurretIO io;


    private double accelRad = 0.0;

  private double targetAngle = 0.0;
  private double voltageCmd = 0.0;
    private boolean reachedTargetPosition = true;
    private double voltageCmdManual = 0.0;
    private boolean hasHitRightLimitSwitch = false;
    private boolean hasHitLeftLimitSwitch = false;

    private boolean isAuto = false;

    public enum TurretMode{
      MANUAL,
      GO_TO_POSITION
    }

    private TurretMode turretMode;

    public Turret(TurretIO io){
        this.io = io;
        reachedTargetPosition = true;
        this.turretMode = TurretMode.GO_TO_POSITION;
        setTargetPosition(getAngle());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        switch (turretMode) {
          case GO_TO_POSITION:
            voltageCmd = 0.0;
            voltageCmdManual = 0.0;
            break;
          case MANUAL:
            voltageCmd = 0.0;
            setTargetPosition(getAngle());
            break;
        }

        if (inputs.rightLimitSwitch) {
          hasHitRightLimitSwitch = true;
        }
        if (inputs.leftLimitSwitch) {
          hasHitLeftLimitSwitch = true;
        }

        io.setVoltage(voltageCmdManual + voltageCmd);
    }


    public void setTargetPosition(double targetAngle) {
        this.targetAngle = targetAngle;
        
        reachedTargetPosition = false;
      }
        public Command runGoToPositionCommand(double targetAngle){
    turretMode = TurretMode.GO_TO_POSITION;
    if ((targetAngle > TurretConstants.TURRET_MAX_ANGLE || targetAngle < TurretConstants.TURRET_MIN_ANGLE)) {
      System.out.println("Soft Limited Turret");
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetAngle );
    isAuto = DriverStation.isAutonomous();
    return new InstantCommand(() -> setTargetPosition(targetAngle), this);
  }




      public void runManualPosition(double stickPosition){
        if((inputs.leftLimitSwitch && stickPosition > Constants.JOYSTICK_DEADBAND) || (inputs.rightLimitSwitch && stickPosition < Constants.JOYSTICK_DEADBAND)){
          turretMode = TurretMode.GO_TO_POSITION;
        }else if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
          turretMode = TurretMode.MANUAL;
          voltageCmdManual = -stickPosition * TurretConstants.TURRET_MANUAL_SCALAR;
        }
        else{
          turretMode = TurretMode.GO_TO_POSITION;
          }
      } 
        
       
  @AutoLogOutput
  public TurretMode getTurretMode(){
    return turretMode;
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
  public double getVoltageCommandPid() {
    return voltageCmd;
  }

  public void setVoltage(double volts) {
    this.voltageCmd = volts;
  }

  @AutoLogOutput
  public double getAppliedVoltage(){
    return inputs.appliedVolts;
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