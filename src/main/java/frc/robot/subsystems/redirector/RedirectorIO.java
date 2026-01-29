
package frc.robot.subsystems.redirector;

import org.littletonrobotics.junction.AutoLog;

public interface RedirectorIO {
    @AutoLog
    public static class RedirectorIOInputs{
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double velocityRadPerSec = 0.0;
        public double currentAngle = 0.0;
    }
      /** Updates the set of loggable inputs. */
  public default void updateInputs(RedirectorIOInputs inputs) {};
  public default void setVoltage(double voltage) {};


}
