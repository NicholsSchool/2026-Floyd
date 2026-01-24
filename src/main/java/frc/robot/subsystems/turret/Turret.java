package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;


public class Turret extends SubsystemBase
{
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private TurretIO io;


    private double accelRad = 0.0;

    private double targetAngle = 0.0;
    private double voltageCmdPid = 0.0;
    private boolean reachedTargetPosition = true;
    private double voltageCmdManual = 0.0;
    private boolean hasHitLimitSwitch = false;
    private boolean isAuto = false;

    private enum TurretMode{
      MANUAL,
      GO_TO_POSITION
    }

    private TurretMode turretMode;

    private final ProfiledPIDController turretPidController =
    new ProfiledPIDController(
        0,0 ,0, new TrapezoidProfile.Constraints(0,0));

        
  private static final LoggedTunableNumber turretMaxVelocityRad =
    new LoggedTunableNumber("turret/MaxVelocityRad");
  private static final LoggedTunableNumber turretMaxAccelerationRad =
    new LoggedTunableNumber("turret/MaxAccelerationRad");
  private static final LoggedTunableNumber turretKp = new LoggedTunableNumber("turret/Kp");
  private static final LoggedTunableNumber turretKi = new LoggedTunableNumber("turret/Ki");
  private static final LoggedTunableNumber turretKd = new LoggedTunableNumber("turret/Kd");

    public Turret(TurretIO io){
        this.io = io;

        reachedTargetPosition = true;

        turretMaxAccelerationRad.initDefault(TurretConstants.kTurretMaxAccelRad);
        turretMaxVelocityRad.initDefault(TurretConstants.kTurretMaxVelRad);


        turretKp.initDefault(TurretConstants.kTurretP);
        turretKi.initDefault(TurretConstants.kTurretI);
        turretKd.initDefault(TurretConstants.kTurretD);

        turretPidController.setP(turretKp.get());
        turretPidController.setI(turretKi.get());
        turretPidController.setD(turretKd.get());

        this.turretMode = TurretMode.GO_TO_POSITION;
        setTargetPosition(getAngle());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        updateTunables();

        Logger.processInputs("Turret", inputs);

        switch(turretMode){
          case GO_TO_POSITION:
          voltageCmdPid = (hasHitLimitSwitch || isAuto) ? turretPidController.calculate( this.getAngle()) : 0.0;
          voltageCmdManual = 0.0;
          break;
          case MANUAL:
          voltageCmdPid = 0.0;
          setTargetPosition(getAngle());
          }

         if (!reachedTargetPosition) {
            reachedTargetPosition = turretPidController.atGoal();
            if (reachedTargetPosition) System.out.println("elevator Move to Pos Reached Goal!");
          }

          if(inputs.limitSwitch){
            hasHitLimitSwitch = true;
          }
        
          io.setVoltage(voltageCmdManual + voltageCmdPid);
    }


    private void updateTunables() {
        // Update from tunable numbers
        if (turretMaxVelocityRad.hasChanged(hashCode())
            || turretMaxAccelerationRad.hasChanged(hashCode())
            || turretKp.hasChanged(hashCode())
            || turretKi.hasChanged(hashCode())
            || turretKd.hasChanged(hashCode())) {
          turretPidController.setP(turretKp.get());
          turretPidController.setI(turretKi.get());
          turretPidController.setD(turretKd.get());
          turretPidController.setConstraints(
              new TrapezoidProfile.Constraints(turretMaxVelocityRad.get(), turretMaxAccelerationRad.get()));
        }
      }

      public void setTargetPosition(double targetAngle) {
        this.targetAngle = targetAngle;
        turretPidController.setGoal((targetAngle));
        turretPidController.reset(inputs.currentAngle);
        reachedTargetPosition = false;
      }

      // Checks if TargetAngle is valid
        public Command runGoToPositionCommand(double targetAngle){
    turretMode = TurretMode.GO_TO_POSITION;
    if ((targetAngle > TurretConstants.kTurretMaxAngle || targetAngle < TurretConstants.kTurretMinAngle) && hasHitLimitSwitch) {
      System.out.println("Soft Limited Turret");
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetAngle );
    isAuto = DriverStation.isAutonomous();
    return new InstantCommand(() -> setTargetPosition(targetAngle), this);
  }

    /**
     * When the Joystick passes the Joystick Deadband threshold
     * the elevator is set to manual else it is go to position
    */
      public void runManualPosition(double stickPosition){
        if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
          turretMode = TurretMode.MANUAL;
        }
        else{
          turretMode = TurretMode.GO_TO_POSITION;
          }
        }
        
       /** 
        * Autonomous Log Outputs 
        */ 
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
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return turretPidController.atGoal();
  }
  
}