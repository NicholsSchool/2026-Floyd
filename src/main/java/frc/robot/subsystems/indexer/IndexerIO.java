package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        //TODO change these values later
        public double indexerMotorVoltage = 0.0;
        public double indexerSupplyVoltage = 0.0;
        public double indexerCurrentAmps = 0.0;

    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setVoltage(double voltage) {}

}
