package frc.robot.commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

public class AutoAim extends InstantCommand {
  private Drive drive;
  private Shooter shooter;
  private Redirector redirector;
  private Turret turret;

  public AutoAim(Drive drive, Shooter shooter, Redirector redirector, Turret turret) {
    this.drive = drive;
    this.shooter = shooter;
    this.redirector = redirector;
    this.turret = turret;
    hasRequirement(turret);
    hasRequirement(shooter);
    hasRequirement(redirector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    var currentPose = drive.getPose();
    Translation2d hubOffset = (AllianceFlipUtil.apply(FieldConstants.Tower.centerPoint)).minus(currentPose.getTranslation());
    double distance = hubOffset.getNorm();
    double turretAngle = Math.atan2(hubOffset.getY(), hubOffset.getX()) - currentPose.getRotation().getRadians();

    //https://www.desmos.com/calculator/fwhxwn9toz
    double hoodTheta = 40.0048 / Math.pow(distance + 5.88781 , 2) + 0.55959;
    double shooterMetersPerSec = 12.86838 * Math.sin(-1.5504 * Math.pow(distance - 0.184152, 0.2)) + 19.19883;

    double shooterRPM = shooterMetersPerSec * 60 / (2 * Math.PI * ShooterConstants.SHOOTER_RADIUS_METERS);

    redirector.setTargetPosition(hoodTheta);
    turret.setTargetPosition(turretAngle);
    shooter.setRPM(shooterRPM);

  }
}
