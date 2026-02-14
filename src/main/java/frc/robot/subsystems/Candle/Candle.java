// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Candle;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Candle extends SubsystemBase {

    CandleIO io;

    public enum Subsystem{
        DRIVE,
        SHOOTER,
        INDEXER,
        REDIRECTOR,
        INTAKE,
        CLIMBER,
        TURRET
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Candle(CandleIO io) {
        this.io = io;
        io.setColor(CandleConstants.FLOYD_PINK, 0, CandleConstants.LED_MAX);
    }

    public void setColor(RGBWColor color, Subsystem subsystem){
        switch(subsystem){
            case DRIVE:
            io.setColor(color, CandleConstants.DRIVE_START_INDEX, CandleConstants.DRIVE_STOP_INDEX);
            break;

            case SHOOTER:
            io.setColor(color,CandleConstants.SHOOTER_START_INDEX, CandleConstants.SHOOTER_STOP_INDEX);
            break;

            case INDEXER:
            io.setColor(color,CandleConstants.INDEXER_START_INDEX, CandleConstants.INDEXER_STOP_INDEX);
            break;

            case REDIRECTOR:
            io.setColor(color,CandleConstants.REDIRECTOR_START_INDEX, CandleConstants.REDIRECTOR_STOP_INDEX);
            break;

            case INTAKE:
            io.setColor(color,CandleConstants.INTAKE_START_INDEX, CandleConstants.INTAKE_STOP_INDEX);
            break;

            case CLIMBER:
            io.setColor(color,CandleConstants.CLIMBER_START_INDEX, CandleConstants.CLIMBER_STOP_INDEX);
            break;

            case TURRET:
            io.setColor(color,CandleConstants.TURRET_START_INDEX, CandleConstants.TURRET_STOP_INDEX);
            break;
        }
    }
    
}