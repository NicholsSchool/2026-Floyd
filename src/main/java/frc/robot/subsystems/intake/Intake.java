package frc.robot.subsystems.intake;

import javax.sound.midi.VoiceStatus;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final ProfiledPIDController dropperPIDController =
        new ProfiledPIDController(IntakeConstants.DROPPER_P, IntakeConstants.DROPPER_I, IntakeConstants.DROPPER_D,
        new TrapezoidProfile.Constraints(IntakeConstants.DROPPER_MAX_ACCEL, IntakeConstants.DROPPER_MAX_VEL));

    public enum DropState {
        IN,
        OUT
    }

    public Intake(IntakeIO io) {
        this.io = io;
        dropperPIDController.setGoal(IntakeConstants.DROPPER_IN_ANGLE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        
        io.setDropperMotorVoltage(dropperPIDController.calculate(inputs.dropperAngleRadians));
    }

    public void intake() {
        io.setWheelMotorVoltage(IntakeConstants.INTAKE_VOLTAGE);
    }
    public void outtake() {
        io.setWheelMotorVoltage(IntakeConstants.OUTTAKE_VOLTAGE);
    }
    
    public void setDropperGoal(double goal) {
        dropperPIDController.setGoal(goal);
    }

    public void setDropState(DropState state) {
        switch (state) {
            case IN:
                setDropperGoal(IntakeConstants.DROPPER_IN_ANGLE);
                break;
            case OUT:
                setDropperGoal(IntakeConstants.DROPPER_OUT_ANGLE);
                break;
            default:
                break;
        }
    }


}
