package frc.robot.subsystems.Candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class CandleIOReal implements CandleIO {

    private CANdle candle;

    public CandleIOReal(){
        candle = new CANdle(Constants.CAN.CANDLE);
                var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRBW;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);
    }
    @Override
    public void updateInputs(CandleIOInputs inputs){

    }

    @Override
      public void setColor(RGBWColor color, int startingIndex, int stopIndex){
            candle.setControl(new SolidColor(startingIndex, stopIndex).withColor(color));
       }

}
