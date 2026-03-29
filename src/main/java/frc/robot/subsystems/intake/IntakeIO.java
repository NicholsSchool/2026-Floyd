package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAngleRadians = 0.0;

        public double wheelMotorVoltage = 0.0;
        public double pivotMotorVoltage = 0.0;

        public double wheelMotorCurrent = 0.0;
        public double pivotMotorCurrent = 0.0;
    }
    
    
    /** Updates the set of loggable inputs. */
    public void updateInputs(IntakeIOInputs inputs);
    
    /**
     * Set the voltage for the motor(s) that drives the intake wheels
     * @param volts The voltage to set the motor to.
     */
    public void setWheelMotorVoltage(double volts);

    /**
     * Set the voltage for the motor(s) that drop down the intake (and shift the hopper)
     * @param volts The voltage to set the motor to.
     */
    public void setPivotMotorVoltage(double volts);
}
