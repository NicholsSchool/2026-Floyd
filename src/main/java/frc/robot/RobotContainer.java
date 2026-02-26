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
import edu.wpi.first.networktables.GenericEntry;
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
    ShuffleboardTab tuningTab;

    // Tuning entries — redirector
    GenericEntry entryRedirectorKp;
    GenericEntry entryRedirectorKi;
    GenericEntry entryRedirectorKd;
    GenericEntry entryRedirectorMaxVel;
    GenericEntry entryRedirectorMaxAccel;

    // Tuning entries — turret
    GenericEntry entryTurretKp;
    GenericEntry entryTurretKi;
    GenericEntry entryTurretKd;
    GenericEntry entryTurretMaxVel;
    GenericEntry entryTurretMaxAccel;

    // Tuning entries — shooter
    GenericEntry entryShooterKp;
    GenericEntry entryShooterKi;
    GenericEntry entryShooterKd;
    GenericEntry entryShooterBangBangMult;
    

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
                new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

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
                new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

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
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));

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
    tuningTab = Shuffleboard.getTab("Tuning");

    // ── Redirector column (col 0) ────────────────────────────────────────────
    entryRedirectorKp = tuningTab.add("Redirector Kp",
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_P)
        .withPosition(0, 0).withSize(2, 1).getEntry();
    entryRedirectorKi = tuningTab.add("Redirector Ki",
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_I)
        .withPosition(0, 1).withSize(2, 1).getEntry();
    entryRedirectorKd = tuningTab.add("Redirector Kd",
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_D)
        .withPosition(0, 2).withSize(2, 1).getEntry();
    entryRedirectorMaxVel = tuningTab.add("Redirector MaxVel",
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_MAX_VEL_RAD)
        .withPosition(0, 3).withSize(2, 1).getEntry();
    entryRedirectorMaxAccel = tuningTab.add("Redirector MaxAccel",
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD)
        .withPosition(0, 4).withSize(2, 1).getEntry();

    // ── Turret column (col 2) ────────────────────────────────────────────────
    entryTurretKp = tuningTab.add("Turret Kp",
        frc.robot.subsystems.turret.TurretConstants.TURRET_P)
        .withPosition(2, 0).withSize(2, 1).getEntry();
    entryTurretKi = tuningTab.add("Turret Ki",
        frc.robot.subsystems.turret.TurretConstants.TURRET_I)
        .withPosition(2, 1).withSize(2, 1).getEntry();
    entryTurretKd = tuningTab.add("Turret Kd",
        frc.robot.subsystems.turret.TurretConstants.TURRET_D)
        .withPosition(2, 2).withSize(2, 1).getEntry();
    entryTurretMaxVel = tuningTab.add("Turret MaxVel",
        frc.robot.subsystems.turret.TurretConstants.TURRET_MAX_VEL_RAD)
        .withPosition(2, 3).withSize(2, 1).getEntry();
    entryTurretMaxAccel = tuningTab.add("Turret MaxAccel",
        frc.robot.subsystems.turret.TurretConstants.TURRET_MAX_ACCEL_RAD)
        .withPosition(2, 4).withSize(2, 1).getEntry();

    // ── Shooter column (col 4) ───────────────────────────────────────────────
    entryShooterKp = tuningTab.add("Shooter Kp",
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_P)
        .withPosition(4, 0).withSize(2, 1).getEntry();
    entryShooterKi = tuningTab.add("Shooter Ki",
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_I)
        .withPosition(4, 1).withSize(2, 1).getEntry();
    entryShooterKd = tuningTab.add("Shooter Kd",
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_D)
        .withPosition(4, 2).withSize(2, 1).getEntry();
    entryShooterBangBangMult = tuningTab.add("Shooter BangBangMult",
        frc.robot.subsystems.shooter.ShooterConstants.BANG_BANG_MULT)
        .withPosition(4, 3).withSize(2, 1).getEntry();
  }

  public void updateShuffleboard(){
    edu.wpi.first.networktables.NetworkTableInstance nt =
        edu.wpi.first.networktables.NetworkTableInstance.getDefault();

    nt.getEntry("/Tuning/redirector/Kp").setDouble(entryRedirectorKp.getDouble(
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_P));
    nt.getEntry("/Tuning/redirector/Ki").setDouble(entryRedirectorKi.getDouble(
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_I));
    nt.getEntry("/Tuning/redirector/Kd").setDouble(entryRedirectorKd.getDouble(
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_D));
    nt.getEntry("/Tuning/redirector/MaxVelocityRad").setDouble(entryRedirectorMaxVel.getDouble(
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_MAX_VEL_RAD));
    nt.getEntry("/Tuning/redirector/MaxAccelerationRad").setDouble(entryRedirectorMaxAccel.getDouble(
        frc.robot.subsystems.redirector.RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD));

    nt.getEntry("/Tuning/turret/Kp").setDouble(entryTurretKp.getDouble(
        frc.robot.subsystems.turret.TurretConstants.TURRET_P));
    nt.getEntry("/Tuning/turret/Ki").setDouble(entryTurretKi.getDouble(
        frc.robot.subsystems.turret.TurretConstants.TURRET_I));
    nt.getEntry("/Tuning/turret/Kd").setDouble(entryTurretKd.getDouble(
        frc.robot.subsystems.turret.TurretConstants.TURRET_D));
    nt.getEntry("/Tuning/turret/MaxVelocityRad").setDouble(entryTurretMaxVel.getDouble(
        frc.robot.subsystems.turret.TurretConstants.TURRET_MAX_VEL_RAD));
    nt.getEntry("/Tuning/turret/MaxAccelerationRad").setDouble(entryTurretMaxAccel.getDouble(
        frc.robot.subsystems.turret.TurretConstants.TURRET_MAX_ACCEL_RAD));

    nt.getEntry("/Tuning/shooter/Kp").setDouble(entryShooterKp.getDouble(
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_P));
    nt.getEntry("/Tuning/shooter/Ki").setDouble(entryShooterKi.getDouble(
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_I));
    nt.getEntry("/Tuning/shooter/Kd").setDouble(entryShooterKd.getDouble(
        frc.robot.subsystems.shooter.ShooterConstants.VELOCITY_D));
    nt.getEntry("/Tuning/shooter/BangBangMult").setDouble(entryShooterBangBangMult.getDouble(
        frc.robot.subsystems.shooter.ShooterConstants.BANG_BANG_MULT));
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
      if (shooter != null) shooter.setDefaultCommand(new ShooterAutoAim(drive, shooter));

      candle.setDefaultCommand(new CandleUpdate(candle, drive, intake, turret, redirector, shooter, indexer).repeatedly());

    
      driveController.a().whileTrue(new InstantCommand(()-> intake.setWheelVoltage(1)));
      driveController.b().whileTrue(new InstantCommand(()-> indexer.setVoltage(1)));
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
