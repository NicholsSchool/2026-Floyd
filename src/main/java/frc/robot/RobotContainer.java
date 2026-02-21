// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.CandleUpdate;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle.CandleIOReal;
import frc.robot.subsystems.Candle.CandleIOSim;
import frc.robot.commands.RedirectorAutoAim;
import frc.robot.commands.ShooterAutoAim;
import frc.robot.commands.TurretAutoAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONAVX;
import frc.robot.subsystems.drive.GyroIORedux;
import frc.robot.subsystems.drive.ModuleIOMaxSwerve;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.redirector.RedirectorIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOFrankenlew;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    Drive drive;
    Vision vision;
    Turret turret;
    Redirector redirector;
    Intake intake;
    Shooter shooter;
    Indexer indexer;
    Candle candle;

    // shuffleboard
    ShuffleboardTab shuffleboardTab;
    

  // Controllers
    public static CommandXboxController driveController = new CommandXboxController(0);
    public static CommandXboxController operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        // Real robot, instantiate hardware IO implementations
        //pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kRev);
        //colorInfo = new ColorInfo();
        drive =
            new Drive(
                new GyroIORedux(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        vision =
             new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0));

        redirector = new Redirector(new RedirectorIOSim());
        turret = new Turret(new TurretIOSim());
        indexer = new Indexer( new IndexerIOSim());
        candle = new Candle(new CandleIOReal());
        break;

        
      case ROBOT_REAL_FRANKENLEW:
        // Real robot, instantiate hardware IO implementations
        //pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kRev);
        //colorInfo = new ColorInfo();
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOMaxSwerve(0),
                new ModuleIOMaxSwerve(1),
                new ModuleIOMaxSwerve(2),
                new ModuleIOMaxSwerve(3));
        vision =
              new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0));

        redirector = new Redirector(new RedirectorIOSim());
        turret = new Turret(new TurretIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOFrankenlew());
        shooter = new Shooter(new ShooterIOSim());
        candle = new Candle(new CandleIOReal());

        break;

      case ROBOT_SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0, drive::getPose));

        redirector = new Redirector(new RedirectorIOSim());
        turret = new Turret(new TurretIOSim());                
        indexer = new Indexer( new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());

        shooter = new Shooter(new ShooterIOSim());

        candle = new Candle(new CandleIOSim());
        break;
    }

    // Configure the trigger bindings
    configureBindings();

    initShuffleboard();
  }

  private void initShuffleboard() {
    shuffleboardTab = Shuffleboard.getTab("Floyd");
  }

  public void updateShuffleboard(){

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
          drive,
          () -> -driveController.getLeftY() * DriveConstants.LOW_GEAR_SCALER,
          () -> -driveController.getLeftX() * DriveConstants.LOW_GEAR_SCALER,
          () -> -driveController.getRightX() * DriveConstants.TURNING_SCALAR,
          () -> Constants.DRIVE_ROBOT_RELATIVE));

      turret.setDefaultCommand(new TurretAutoAim(drive, turret));
      redirector.setDefaultCommand(new RedirectorAutoAim(drive, redirector));
      shooter.setDefaultCommand(new ShooterAutoAim(drive, shooter));

      candle.setDefaultCommand(new CandleUpdate(candle, drive, intake, turret, redirector, shooter, indexer).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return new InstantCommand();
  }
}
