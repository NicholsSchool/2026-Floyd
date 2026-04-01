package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        //TODO change these values later
        public double indexerLeftVoltage = 0.0;
        public double indexerRightVoltage = 0.0;
        public double indexerLeftSupplyVoltage = 0.0;
        public double indexerRightSupplyVoltage = 0.0;
        public double indexerLeftCurrentAmps = 0.0;
        public double indexerRightCurrentAmps = 0.0;
        public boolean hasBall = false;

    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setVoltageIndexer(double voltage) {}
    public default void setVoltageFeeder(double voltage) {}



}
