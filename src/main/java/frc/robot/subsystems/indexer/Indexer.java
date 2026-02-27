package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class  Indexer extends SubsystemBase {
  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Indexer/SpinDurationSec");
  private static final LoggedTunableNumber indexVoltage = new LoggedTunableNumber("Indexer/indexVoltage");
  private static final LoggedTunableNumber reverseVoltage = new LoggedTunableNumber("Indexer/reverseVoltage");


  public Indexer(IndexerIO io) {
    System.out.println("[Init] Creating Indexer");
    this.io = io;


    // Sets the default using IndexerConstants // MAKE Indexer CONSTANTS!!! Simply fill-in
    indexVoltage.initDefault(IndexerConstants.INDEXER_VOLTAGE);
    reverseVoltage.initDefault(IndexerConstants.REVERSE_VOLTAGE);
    spinDurationSec.initDefault(1.5);
      
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    }
  


  public void setIndexVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stopIndexer(){
    setIndexVoltage(0);
  }

  public void index() {
        setIndexVoltage(IndexerConstants.INDEXER_VOLTAGE);
    }
  public void outdex() {
        setIndexVoltage(IndexerConstants.REVERSE_VOLTAGE);
    }

  public boolean hasBall(){
    return inputs.hasBall;
  }

  @AutoLogOutput
  public double getVoltage() {
    return inputs.indexerVoltage;
  }

}
