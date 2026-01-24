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
  private double setpoint;
  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  public double voltageCommand;
  private IndexerMode mode;


  public static final LoggedTunableNumber shootVelocity =
      new LoggedTunableNumber("Indexer/AmpVelocityRPMs");
  public static final LoggedTunableNumber deliverVelocity =
      new LoggedTunableNumber("Indexer/SpeakerVelocityRPMs");
  public static final LoggedTunableNumber reverseVeloctiy =
      new LoggedTunableNumber("Indexer/TrapVelocityRPMs");
  public static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Indexer/SpinDurationSec");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD");


  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;

  public Indexer(IndexerIO io) {
    System.out.println("[Init] Creating Indexer");
    this.io = io;


    // Sets the default using IndexerConstants // MAKE Indexer CONSTANTS!!! Simply fill-in
    indexVelocity.initDefault(Constants.IndexerConstants.Indexer_RPM);
    reverseVeloctiy.initDefault(Constants.IndexerConstants.REVERSE_RPM);
    spinDurationSec.initDefault(1.5);
    kP.initDefault(Constants.IndexerConstants.P);
    kD.initDefault(Constants.IndexerConstants.D);


    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.001, 0.0060);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(1, 1);
        break;
    }
  }

  @Override
  public void periodic() {
    setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);


    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get()); //kD, and kP may also be K and D inside Indexer constants
      controller.setD(kD.get());
    }


    // Reset when disabled
    if (DriverStation.isDisabled()) {
      controller.reset();
      mode = IndexerMode.Stopped;
    } else {
      switch (mode) {
        case Go:
          setpoint = shootVelocity.get();
          break;
        case Reverse:
          setpoint = reverseVeloctiy.get();
          break;
        case Stopped:
        default:
          setpoint = 0.0;
      }


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
  }


  public void setReverse() {
    mode = IndexerMode.Reverse;
  }


  public void stop() {
    mode = IndexerMode.Stopped;
  }


  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }


  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  public void setVelocityRPMs(double velocityRPMs) {
    this.setpoint = velocityRPMs;
  }

  @AutoLogOutput
  public double getSetpointRPMs() {
    return setpoint;
  }


  @AutoLogOutput
  public double getActualVelocityRPMs() {
    return inputs.velocityRPMs;
  }
}
