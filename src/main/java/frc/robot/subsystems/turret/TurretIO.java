package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double velocityRadPerSec = 0.0;
        public double currentAngle = 0.0;

    }
      /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {};
  public default void setVoltage(double voltage) {};


}