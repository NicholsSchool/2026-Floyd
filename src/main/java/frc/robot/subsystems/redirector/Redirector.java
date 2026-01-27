package frc.robot.subsystems.redirector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.redirector.RedirectorConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.LoggedTunableNumber;


public class Redirector extends SubsystemBase
{
    private final RedirectorIOInputsAutoLogged inputs = new RedirectorIOInputsAutoLogged();
    private RedirectorIO io;


    private double accelRad = 0.0;

    private double targetAngle = 0.0;
    private double voltageCmdPid = 0.0;
    private boolean reachedTargetPosition = true;
    private double voltageCmdManual = 0.0;
    private boolean isAuto = false;

    private enum RedirectorMode{
      MANUAL,
      GO_TO_POSITION
    }

    private RedirectorMode redirectorMode;

    private final ProfiledPIDController redirectorPidController =
    new ProfiledPIDController(
        0,0 ,0, new TrapezoidProfile.Constraints(0,0));

        
  private static final LoggedTunableNumber redirectorMaxVelocityRad =
    new LoggedTunableNumber("redirector/MaxVelocityRad");
  private static final LoggedTunableNumber redirectorMaxAccelerationRad =
    new LoggedTunableNumber("redirector/MaxAccelerationRad");
  private static final LoggedTunableNumber redirectorKp = new LoggedTunableNumber("redirector/Kp");
  private static final LoggedTunableNumber redirectorKi = new LoggedTunableNumber("redirector/Ki");
  private static final LoggedTunableNumber redirectorKd = new LoggedTunableNumber("redirector/Kd");

    public Redirector(RedirectorIO io){
        this.io = io;

        reachedTargetPosition = true;

        redirectorMaxAccelerationRad.initDefault(RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD);
        redirectorMaxVelocityRad.initDefault(RedirectorConstants.REDIRECTOR_MAX_VEL_RAD);


        redirectorKp.initDefault(RedirectorConstants.REDIRECTOR_P);
        redirectorKi.initDefault(RedirectorConstants.REDIRECTOR_I);
        redirectorKd.initDefault(RedirectorConstants.REDIRECTOR_D);

        redirectorPidController.setP(redirectorKp.get());
        redirectorPidController.setI(redirectorKi.get());
        redirectorPidController.setD(redirectorKd.get());

        this.redirectorMode = RedirectorMode.GO_TO_POSITION;
        setTargetPosition(getAngle());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        updateTunables();

        Logger.processInputs("Redirector", inputs);

        switch(redirectorMode){
          case GO_TO_POSITION:
          voltageCmdPid = redirectorPidController.calculate( this.getAngle());
          voltageCmdManual = 0.0;
          break;
          case MANUAL:
          voltageCmdPid = 0.0;
          setTargetPosition(getAngle());
          }

         if (!reachedTargetPosition) {
            reachedTargetPosition = redirectorPidController.atGoal();
            if (reachedTargetPosition) System.out.println("redirector Move to Pos Reached Goal!");
          }
        
          io.setVoltage(voltageCmdManual + voltageCmdPid);
    }


    private void updateTunables() {
        // Update from tunable numbers
        if (redirectorMaxVelocityRad.hasChanged(hashCode())
            || redirectorMaxAccelerationRad.hasChanged(hashCode())
            || redirectorKp.hasChanged(hashCode())
            || redirectorKi.hasChanged(hashCode())
            || redirectorKd.hasChanged(hashCode())) {
          redirectorPidController.setP(redirectorKp.get());
          redirectorPidController.setI(redirectorKi.get());
          redirectorPidController.setD(redirectorKd.get());
          redirectorPidController.setConstraints(
              new TrapezoidProfile.Constraints(redirectorMaxVelocityRad.get(), redirectorMaxAccelerationRad.get()));
        }
      }

      public void setTargetPosition(double targetAngle) {
        this.targetAngle = targetAngle;
        redirectorPidController.setGoal((targetAngle));
        redirectorPidController.reset(inputs.currentAngle);
        reachedTargetPosition = false;
      }

      // Checks if TargetAngle is valid
        public Command runGoToPositionCommand(double targetAngle){
    redirectorMode = RedirectorMode.GO_TO_POSITION;
    if ((targetAngle > RedirectorConstants.REDIRECTOR_MAX_ANGLE || targetAngle < RedirectorConstants.REDIRECTOR_MIN_ANGLE)) {
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetAngle );
    isAuto = DriverStation.isAutonomous();
    return new InstantCommand(() -> setTargetPosition(targetAngle), this);
  }

    /**
     * When the Joystick passes the Joystick Deadband threshold
     * the redirector is set to manual else it is go to position
    */
       public void runManualPosition(double stickPosition){
        if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
          redirectorMode = RedirectorMode.MANUAL;
          voltageCmdManual = stickPosition * RedirectorConstants.REDIRECTOR_MANUAL_SCALAR;
        }
        else{
          redirectorMode = RedirectorMode.GO_TO_POSITION;
          }
        
      } 
        
       /** 
        * Autonomous Log Outputs 
        */ 
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
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return redirectorPidController.atGoal();
  }
  
}