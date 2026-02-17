package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.IndexerConstants;



public class Indexer extends SubsystemBase {
  private double setpoint = 0.0;
  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  public double voltageCommand;
  private IndexerMode mode;

  public static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Indexer/SpinDurationSec");
      //TODO change these numbers later
  private static final LoggedTunableNumber INDEX_P = new LoggedTunableNumber("Indexer/kP");
  private static final LoggedTunableNumber INDEX_D = new LoggedTunableNumber("Indexer/kD");
  private static final LoggedTunableNumber indexVelocity = new LoggedTunableNumber("Indexer/indexVelocity");
  private static final LoggedTunableNumber reverseVelocity = new LoggedTunableNumber("Indexer/reverseVelocity");


  private PIDController controller = new PIDController(0.0, 0.0, 0.0);

  public Indexer(IndexerIO io) {
    System.out.println("[Init] Creating Indexer");
    this.io = io;


    // Sets the default using IndexerConstants // MAKE Indexer CONSTANTS!!! Simply fill-in
    indexVelocity.initDefault(Constants.IndexerConstants.Indexer_RPM);
    reverseVelocity.initDefault(Constants.IndexerConstants.Reverse_RPM);
    spinDurationSec.initDefault(1.5);
    INDEX_P.initDefault(Constants.IndexerConstants.P);
    INDEX_D.initDefault(Constants.IndexerConstants.D);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);


    // Update tunable numbers
    if (INDEX_P.hasChanged(hashCode()) || INDEX_D.hasChanged(hashCode())) {
      controller.setP(INDEX_P.get()); //kD, and kP may also be K and D inside Indexer constants
      controller.setD(INDEX_D.get());
    }


    // Reset when disabled
    if (DriverStation.isDisabled()) {
      controller.reset();
      mode = IndexerMode.Stopped;
    } else {
      switch (mode) {
        case Go:
          setpoint = indexVelocity.get();
          break;
        case Reverse:
          setpoint = reverseVelocity.get();
          break;
        case Stopped:
        default:
          setpoint = 0.0;
      }

      //TODO change Voltage command calculation later. AKA change these values
      voltageCommand = controller.calculate(inputs.velocityRPMs, setpoint);
      io.setVoltage(MathUtil.clamp(voltageCommand, -12.0, 12.0));
    }
  }

        private static enum IndexerMode {
        Go,
        Stopped,
        Reverse;
      };

  public void setIndex() {
    mode = IndexerMode.Go;
    controller.reset();
  }


  public void setReverse() {
    mode = IndexerMode.Reverse;
    controller.reset();
  }


  public void stop() {
    mode = IndexerMode.Stopped;
    controller.reset();
  }


  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public boolean hasBall(){
    return inputs.hasBall;
  }


  public void setVelocityRPMs(double velocityRPMs) {
    this.setpoint = velocityRPMs;
    inputs.velocityRPMs = velocityRPMs;
    io.setVelocityRPMs(velocityRPMs);
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  @AutoLogOutput
  public double getSetpointRPMs() {
    return setpoint;
  }
}
