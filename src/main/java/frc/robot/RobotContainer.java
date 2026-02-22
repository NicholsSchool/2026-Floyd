// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.redirector.RedirectorConstants;
import frc.robot.subsystems.redirector.RedirectorIOReal;
import frc.robot.commands.CandleUpdate;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Candle.CandleIOReal;
import frc.robot.subsystems.Candle.CandleIOSim;
import frc.robot.commands.RedirectorAutoAim;
import frc.robot.commands.ShooterAutoAim;
import frc.robot.commands.TurretAutoAim;
import frc.robot.subsystems.drive.Drive;
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
import frc.robot.subsystems.shooter.ShooterIOReal;
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
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));

        redirector = new Redirector(new RedirectorIOReal());
        turret = new Turret(new TurretIOSim());
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOReal());
        candle = new Candle(new CandleIOSim());

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

    SmartDashboard.putNumber("Tuning/TurrentToPose/Kp",          TurretConstants.TURRET_P);
    SmartDashboard.putNumber("Tuning/TurrentToPose/Ki",          TurretConstants.TURRET_I);
    SmartDashboard.putNumber("Tuning/TurrentToPose/Kd",          TurretConstants.TURRET_D);
    SmartDashboard.putNumber("Tuning/TurrentToPose/MaxVelRad",   TurretConstants.TURRET_MAX_VEL_RAD);
    SmartDashboard.putNumber("Tuning/TurrentToPose/MaxAccelRad", TurretConstants.TURRET_MAX_ACCEL_RAD);
    SmartDashboard.putNumber("Tuning/TurrentToPose/MinAngle",    TurretConstants.TURRET_MIN_ANGLE);
    SmartDashboard.putNumber("Tuning/TurrentToPose/MaxAngle",    TurretConstants.TURRET_MAX_ANGLE);
    SmartDashboard.putNumber("Tuning/TurrentToPose/OnGoalTolerance", 0.02);
    SmartDashboard.putNumber("Tuning/TurrentToPose/AngleOffset", 0.0);
    SmartDashboard.putNumber("Tuning/TurrentToPose/AimDeadband", Math.toRadians(1.5));

    // Hood
    SmartDashboard.putNumber("Tuning/HoodToPose/Kp",             RedirectorConstants.REDIRECTOR_P);
    SmartDashboard.putNumber("Tuning/HoodToPose/Ki",             RedirectorConstants.REDIRECTOR_I);
    SmartDashboard.putNumber("Tuning/HoodToPose/Kd",             RedirectorConstants.REDIRECTOR_D);
    SmartDashboard.putNumber("Tuning/HoodToPose/MaxVelRad",      RedirectorConstants.REDIRECTOR_MAX_VEL_RAD);
    SmartDashboard.putNumber("Tuning/HoodToPose/MaxAccelRad",    RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD);
    SmartDashboard.putNumber("Tuning/HoodToPose/MinAngle",       RedirectorConstants.REDIRECTOR_MIN_ANGLE);
    SmartDashboard.putNumber("Tuning/HoodToPose/MaxAngle",       RedirectorConstants.REDIRECTOR_MAX_ANGLE);
    SmartDashboard.putNumber("Tuning/HoodToPose/OnGoalTolerance", 0.02);

    tuningTab.add("Turret Kp",          TurretConstants.TURRET_P)         .withPosition(0, 0).withSize(2, 1);
    tuningTab.add("Turret Ki",          TurretConstants.TURRET_I)         .withPosition(0, 1).withSize(2, 1);
    tuningTab.add("Turret Kd",          TurretConstants.TURRET_D)         .withPosition(0, 2).withSize(2, 1);
    tuningTab.add("Turret MaxVel",      TurretConstants.TURRET_MAX_VEL_RAD)  .withPosition(0, 3).withSize(2, 1);
    tuningTab.add("Turret MaxAccel",    TurretConstants.TURRET_MAX_ACCEL_RAD).withPosition(0, 4).withSize(2, 1);
    tuningTab.add("Turret MinAngle",    TurretConstants.TURRET_MIN_ANGLE)  .withPosition(0, 5).withSize(2, 1);
    tuningTab.add("Turret MaxAngle",    TurretConstants.TURRET_MAX_ANGLE)  .withPosition(0, 6).withSize(2, 1);
    tuningTab.add("Turret Tolerance",   0.02)                              .withPosition(0, 7).withSize(2, 1);
    tuningTab.add("Turret AngleOffset", 0.0)                               .withPosition(0, 8).withSize(2, 1);
    tuningTab.add("Turret AimDeadband", Math.toRadians(1.5))               .withPosition(0, 9).withSize(2, 1);

    tuningTab.add("Hood Kp",            RedirectorConstants.REDIRECTOR_P)          .withPosition(3, 0).withSize(2, 1);
    tuningTab.add("Hood Ki",            RedirectorConstants.REDIRECTOR_I)          .withPosition(3, 1).withSize(2, 1);
    tuningTab.add("Hood Kd",            RedirectorConstants.REDIRECTOR_D)          .withPosition(3, 2).withSize(2, 1);
    tuningTab.add("Hood MaxVel",        RedirectorConstants.REDIRECTOR_MAX_VEL_RAD).withPosition(3, 3).withSize(2, 1);
    tuningTab.add("Hood MaxAccel",      RedirectorConstants.REDIRECTOR_MAX_ACCEL_RAD).withPosition(3, 4).withSize(2, 1);
    tuningTab.add("Hood MinAngle",      RedirectorConstants.REDIRECTOR_MIN_ANGLE)  .withPosition(3, 5).withSize(2, 1);
    tuningTab.add("Hood MaxAngle",      RedirectorConstants.REDIRECTOR_MAX_ANGLE)  .withPosition(3, 6).withSize(2, 1);
    tuningTab.add("Hood Tolerance",     0.02)                                      .withPosition(3, 7).withSize(2, 1);
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
    // drive.setDefaultCommand(
    //   DriveCommands.joystickDrive(
    //       drive,
    //       () -> -driveController.getLeftY() * Constants.DriveConstants.LOW_GEAR_SCALER,
    //       () -> -driveController.getLeftX() * Constants.DriveConstants.LOW_GEAR_SCALER,
    //       () -> -driveController.getRightX() * Constants.DriveConstants.TURNING_SCALAR,
    //       () -> Constants.DRIVE_ROBOT_RELATIVE));

      // turret.setDefaultCommand(new TurretAutoAim(drive, turret));
      // redirector.setDefaultCommand(new RedirectorAutoAim(drive, redirector));
      // shooter.setDefaultCommand(new ShooterAutoAim(drive, shooter));

      // candle.setDefaultCommand(new CandleUpdate(candle, drive, intake, turret, redirector, shooter, indexer).repeatedly());
      //shooter.setDefaultCommand(new RepeatCommand(new InstantCommand(() -> shooter.setVoltage(5.0 * driveController.getLeftY()), shooter)));

        driveController.a().whileTrue(new InstantCommand(() -> shooter.setRPM(2800)));
        driveController.b().whileTrue(new InstantCommand(() -> shooter.setRPM(3000)));
        driveController.x().whileTrue(new InstantCommand(() -> shooter.setRPM(2900)));
        driveController.y().whileTrue(new InstantCommand(() -> shooter.stop()));

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
