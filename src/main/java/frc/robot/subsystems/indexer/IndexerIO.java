package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double indexerMotorVoltage = 0.0;
        public double indexerSupplyVoltage = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerPositionIN = 0.0;
        public double indexerVelocityINPerSec = 0.0;

        public boolean position1Occupied = false;
        public boolean position2Occupied = false;
        public boolean position3Occupied = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setMotorVoltage(double voltage) {}

}
