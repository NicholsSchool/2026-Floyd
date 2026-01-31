package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double supplyVoltage = 0.0;
        public double currentAmps = 0.0;
        public double velocityRPM = 0.0;
    }

    /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}
}
