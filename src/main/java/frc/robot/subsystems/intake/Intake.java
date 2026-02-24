package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final ProfiledPIDController pivotPIDController = new ProfiledPIDController(
        IntakeConstants.PIVOT_P, IntakeConstants.PIVOT_I, IntakeConstants.PIVOT_D,
        new TrapezoidProfile.Constraints(IntakeConstants.PIVOT_MAX_ACCEL, IntakeConstants.PIVOT_MAX_VEL));


    /**
     * Preset positions for the intake pivot
     * Use as shorthand in {@link #setPivotGoal(PivotPreset)} for commands.
     * Will be AutoLogged.
     */
    public enum PivotPreset {
        IN("In"),
        OUT("Out"),
        CUSTOM("Custom");

        private final String name;
        private PivotPreset(String name) {
            this.name = name;
        }

        @Override
        public String toString() { return name; }

    }

    /**
     * State of the intake pivot
     * If GOTOANGLE, voltage is dictated by PID controller.
     * Will be AutoLogged.
     */
    public enum PivotState {
        MANUAL("Manual"),
        GOTOANGLE("GoToAngle");

        private final String name;
        private PivotState(String name) {
            this.name = name;
        }
        
        @Override
        public String toString() { return name; }
    }

    private PivotPreset pivotPreset = PivotPreset.IN;
    private PivotState pivotState = PivotState.GOTOANGLE;

    public Intake(IntakeIO io) {
        this.io = io;
        setPivotGoal(IntakeConstants.PIVOT_IN_ANGLE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        
        switch (pivotState) {
            case GOTOANGLE:
                io.setPivotMotorVoltage(pivotPIDController.calculate(inputs.pivotAngleRadians));
                break;

            case MANUAL:
                resetPivotPID();
                break;
        
            default:
                break;
        }

    }

    public void setWheelVoltage(double volts) {
        io.setWheelMotorVoltage(volts);
    }

    public void stopWheels() {
        setWheelVoltage(0.0);
    }

    public void intake() {
        setWheelVoltage(IntakeConstants.INTAKE_VOLTAGE);
    }
    public void outtake() {
        setWheelVoltage(IntakeConstants.OUTTAKE_VOLTAGE);
    }

    public void setPivotVoltage(double volts) {
        pivotState = PivotState.MANUAL;
        io.setPivotMotorVoltage(volts);
    }

    public void setPivotGoal(PivotPreset state) {
        switch (state) {
            case IN:
                setPivotGoal(IntakeConstants.PIVOT_IN_ANGLE);
                pivotPreset = PivotPreset.IN;
                break;
            case OUT:
                setPivotGoal(IntakeConstants.PIVOT_OUT_ANGLE);
                pivotPreset = PivotPreset.OUT;
                break;
            default:
                break;
        }
    }
   
    public void setPivotGoal(double goal) {
        pivotPreset = PivotPreset.CUSTOM;
        pivotState = PivotState.GOTOANGLE;
        pivotPIDController.setGoal(goal);
    }


    public void resetPivotPID() {
        pivotPreset = PivotPreset.CUSTOM;
        pivotPIDController.reset(inputs.pivotAngleRadians);
    }


    @AutoLogOutput
    public PivotPreset getPivotPreset() { return pivotPreset; }

    @AutoLogOutput
    public PivotState getPivotState() { return pivotState; }

    @AutoLogOutput
    public double getPivotGoal() { return pivotPIDController.getGoal().position; }
    public double getPivotAngle() { return inputs.pivotAngleRadians; }

    @AutoLogOutput
    public double getPivotVoltage() { return inputs.pivotMotorVoltage; }

    @AutoLogOutput
    public double getWheelVoltage() { return inputs.wheelMotorVoltage; }


}
