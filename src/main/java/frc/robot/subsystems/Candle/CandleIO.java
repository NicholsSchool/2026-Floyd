package frc.robot.subsystems.Candle;

import com.ctre.phoenix6.signals.RGBWColor;

public interface CandleIO {
    public static class CandleIOInputs{

    }
    
    public default void updateInputs(CandleIOInputs inputs) {}
    public default void setColor(RGBWColor color, int startIndex, int stopIndex) {};
}
