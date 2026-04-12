package frc.robot.commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;

public class ShooterAutoAim extends InstantCommand {
  private Drive drive;
  private Shooter shooter;
  private Redirector redirector;

  public ShooterAutoAim(Drive drive, Shooter shooter, Redirector redirector) {
    this.drive = drive;
    this.shooter = shooter;
    this.redirector = redirector;
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

    shooter.setRPM((500 * (distance + AutoConstants.AUTOAIM_GASLIGHT)) + 1800);

  }
}
