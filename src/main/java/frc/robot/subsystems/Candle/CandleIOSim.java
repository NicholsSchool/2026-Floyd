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

public class CandleIOSim implements CandleIO {

    public CandleIOSim(){

    }
    @Override
    public void updateInputs(CandleIOInputs inputs){

    }

    @Override
      public void setColor(RGBWColor color, int startingIndex, int stopIndex){
       }

}
