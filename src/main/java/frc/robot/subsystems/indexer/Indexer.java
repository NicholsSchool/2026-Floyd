package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class  Indexer extends SubsystemBase {
  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private static final LoggedTunableNumber indexVoltage = new LoggedTunableNumber("Indexer/indexVoltage");
  private static final LoggedTunableNumber reverseVoltage = new LoggedTunableNumber("Indexer/reverseVoltage");


  public Indexer(IndexerIO io) {
    System.out.println("[Init] Creating Indexer");
    this.io = io;


    // Sets the default using IndexerConstants // MAKE Indexer CONSTANTS!!! Simply fill-in
    indexVoltage.initDefault(IndexerConstants.INDEXER_VOLTAGE);
    reverseVoltage.initDefault(IndexerConstants.REVERSE_VOLTAGE);
      
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    }
  

  public void stop(){
    io.setVoltageIndexer(0.0);
    io.setVoltageFeeder(0.0);
  }

  public void index() {
      io.setVoltageIndexer(IndexerConstants.INDEXER_VOLTAGE);
    }


  public void backdex() {
        io.setVoltageIndexer(IndexerConstants.REVERSE_VOLTAGE);
    }


     public Command commandIndex() {   
        return new FunctionalCommand(
            () -> System.out.println("indexing"),
            () -> index(),
            interrupted -> stop(),
            () -> false,
            this).withTimeout(10.0);
    } 

  @AutoLogOutput 
  public double getIndexRightVoltage(){
    return inputs.indexerRightVoltage;
  }

  @AutoLogOutput 
  public double getIndexLeftVoltage(){
    return inputs.indexerLeftVoltage;
  }

  @AutoLogOutput
  public double getIndexerCurrent(){
    return inputs.indexerLeftCurrentAmps;
  }


}
