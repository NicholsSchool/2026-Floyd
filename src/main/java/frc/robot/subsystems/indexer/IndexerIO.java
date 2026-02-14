package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        //TODO change these values later
        public double indexerVoltage = 0.0;
        public double velocityRPMs = 0.0;
        public double indexerSupplyVoltage = 0.0;
        public double indexerCurrentAmps = 0.0;
        public boolean hasBall = false;

    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setVoltage(double voltage) {}
    public default void setVelocityRPMs(double velocityRPMs) {}



}
