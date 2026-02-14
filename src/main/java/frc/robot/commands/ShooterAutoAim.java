package frc.robot.commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;

public class ShooterAutoAim extends InstantCommand {
  private Drive drive;
  private Shooter shooter;

  public ShooterAutoAim(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    var currentPose = drive.getPose();
    Translation2d hubOffset = (AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())).minus(currentPose.getTranslation());
    double distance = hubOffset.getNorm();

    //https://www.desmos.com/calculator/fwhxwn9toz
    double shooterMetersPerSec = 12.86838 * Math.sin(-1.5504 * Math.pow(distance - 0.184152, 0.2)) + 19.19883;

    double shooterRPM = shooterMetersPerSec * 60 / (2 * Math.PI * ShooterConstants.SHOOTER_RADIUS_METERS);
    shooter.setRPM(shooterMetersPerSec < 30.0 ? shooterRPM : 30 *  60 / (2 * Math.PI * ShooterConstants.SHOOTER_RADIUS_METERS));

  }
}
