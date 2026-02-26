// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.BradyMathLib;
import frc.robot.util.BradyMathLib.PoseVisionStats;

import java.util.ArrayDeque;

// import frc.robot.commands.VisionCommands.PhotonInfo;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final int kNumModules = 4;
  private final Module[] modules = new Module[kNumModules]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d odometryPose = new Pose2d();
  private Pose2d visionPose = new Pose2d();  
  private Rotation2d lastGyroRotation = new Rotation2d();
  private Pose2d filteredPhotonPose2d = new Pose2d();
  private boolean coastRequest = false;
  private boolean isBrakeModeDrive = true;

  private ArrayDeque<Pose2d> visionStatsBuffer;
  private PoseVisionStats poseVisionStats;

  private Twist2d fieldVelocity = new Twist2d();
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  SwerveModulePosition[] positions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };

  SwerveDrivePoseEstimator kalman =
      new SwerveDrivePoseEstimator(kinematics, lastGyroRotation, positions, odometryPose);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    

    visionStatsBuffer = new ArrayDeque<Pose2d>(VisionConstants.visionStatsNumBuffer);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    } else {
      // Calculate module setpoints
      ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(setpoint, Constants.LOOP_PERIOD_SECS);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

      // Send setpoints to modules
      SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        // The module returns the optimized state, useful for logging
        optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
      }

      // Log setpoint states
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    SwerveModulePosition[] wheelAbsolutes = new SwerveModulePosition[4];
    for (int i = 0; i < kNumModules; i++) {
      wheelAbsolutes[i] = modules[i].getPosition();
    }

    // odometryPose = kalman.getEstimatedPosition();
    updateEstimatedPose(wheelAbsolutes);

    // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < kNumModules; i++) {
      measuredStates[i] = modules[i].getState();
    }
    Logger.recordOutput("SwerveStates/Measured", measuredStates);

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < kNumModules; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }

    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      Rotation2d currentGyroRotation = new Rotation2d(gyroInputs.yawPositionRad);
      twist =
          new Twist2d(twist.dx, twist.dy, currentGyroRotation.minus(lastGyroRotation).getRadians());
      lastGyroRotation = currentGyroRotation;
    } else {
      // no gyro in simulation, faking using odometry twist
      lastGyroRotation = new Rotation2d(twist.dtheta + lastGyroRotation.getRadians());
    }

    odometryPose = odometryPose.exp(twist);

    // Update field velocity
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : chassisSpeeds.omegaRadiansPerSecond);

    // if a coast is requested during tele-op, set the motors to coast when the robot gets disabled
    if( DriverStation.isDisabled() && coastRequest ) {
      for( Module module : modules )
        module.setDriveBrakeMode(false);
      coastRequest = false;
      isBrakeModeDrive = false;
    }

    //Once the robot is reinabled after being on coast in disable, reenable brake mode
    if( DriverStation.isEnabled() && !isBrakeModeDrive ) {
      for( Module module : modules )
        module.setDriveBrakeMode(true);
      isBrakeModeDrive = true;
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    setpoint = speeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry odometryPose. */
  @AutoLogOutput
  public Pose2d getOdomPose() {
    return odometryPose;
  }

  @AutoLogOutput
  public Pose2d getRawLimelightPose(){
    //return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").odometryPose;
    // LimelightHelpers.SetRobotOrientation("limelight", this.getYaw(), 0.0, 0.0, 0.0, 0.0, 0.0 );
    // return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").odometryPose;
    return new Pose2d();
  }

  @AutoLogOutput
  public Pose2d getRawPhotonPose(){
    //return photonCam.getLocalizedPose().toPose2d();
    return visionPose;
  }
  /** returns the filtered extimated pose */
  @AutoLogOutput
  public Pose2d getPose(){
    return kalman.getEstimatedPosition();
  }

  /** Returns the current estimated rotation. */
  @AutoLogOutput
  public Rotation2d getRotation() {
    return kalman.getEstimatedPosition().getRotation();
  }

  public Pose2d getfilteredPhotonPose2d() {
    return filteredPhotonPose2d;
  }

  public void requestCoast() {
    coastRequest = true;
  }

  /** Resets the current odometry odometryPose. */
  /** If using vision for localization, then the robot's pose should not be
   *  set. The pose will immediately be overwritten with the latest vision pose.
   *  So for now, comment out.
   */
  public void setPose(Pose2d odometryPose) {
    System.out.print("Drive::setPose is being called!!");
    //this.odometryPose = odometryPose;
  }

  public void resetFieldHeading() {
    gyroIO.resetIMU();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  /**
   * TJG Returns the measured X, Y, and theta field velocities in meters per sec. The components of
   * the twist are velocities and NOT changes in position.
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns the current yaw velocity (Z rotation) in radians per second. TJG */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }
  /**
   * gets the field velocity in the universal x and y direction
   *
   * @return a twist containing x y and theta
   */
  public Twist2d getFieldOrientedVelocity() {
    return (new Twist2d(
        fieldVelocity.dx * Math.cos(getYaw()) + fieldVelocity.dy * Math.sin(getYaw()),
        fieldVelocity.dx * Math.sin(getYaw()) + fieldVelocity.dy * Math.cos(getYaw()),
        fieldVelocity.dtheta));
  }

  @AutoLogOutput
  public double getYaw() {
    return kalman.getEstimatedPosition().getRotation().getRadians();
  }

  public void updateEstimatedPose(SwerveModulePosition[] wheelAbsolutes) {
    // if (LimelightHelpers.getTV("limelight")) {
    //  //kalman.addVisionMeasurement(new Pose2d(getRawLimelightPose().getTranslation(), new Rotation2d(getYaw())), Timer.getFPGATimestamp()); //trust yaw little, our gyro is much more accurate
    // }      

    // if (usePhotonPose()) {
    //   // kalman.addVisionMeasurement(new Pose2d(getPhotonPose().getTranslation(), new Rotation2d(getYaw())), Timer.getFPGATimestamp());
    //   filteredPhotonPose2d = getRawPhotonPose();
    //   kalman.addVisionMeasurement(filteredPhotonPose2d, Timer.getFPGATimestamp(), getVisionStdDevs(getDistanceToTag()) );
    // }
    kalman.updateWithTime(Timer.getFPGATimestamp(), lastGyroRotation, wheelAbsolutes);
    // System.out.println(photonCam.getArea(photonCam.getBestTarget(photonCam.getLatestPipeline())));
  }

  @SuppressWarnings("unused")
  private void runVisionStats( Pose2d newVisionPose ) {
      //update visionStatsBuffer, keeping the maximun num in at all times, default 100 for testing
      if( visionStatsBuffer.size() >= VisionConstants.visionStatsNumBuffer)
        visionStatsBuffer.removeFirst();
      visionStatsBuffer.addLast(newVisionPose);

      //update stats
      Pose2d meanPose2d = BradyMathLib.getMean(visionStatsBuffer);
      Pose2d stdDevPose2d = BradyMathLib.getStdDevs(visionStatsBuffer, meanPose2d);

      poseVisionStats = new BradyMathLib.PoseVisionStats( meanPose2d, stdDevPose2d );
  }

  @AutoLogOutput
  private PoseVisionStats getVisionStats() {
    return poseVisionStats;
  }


  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

        visionPose = visionRobotPoseMeters;
        kalman.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}
