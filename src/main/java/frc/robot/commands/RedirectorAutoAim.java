package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.util.AllianceFlipUtil;

public class RedirectorAutoAim extends InstantCommand {
  private Drive drive;
  private Redirector redirector;

  public RedirectorAutoAim(Drive drive, Redirector redirector) {
    this.drive = drive;
    this.redirector = redirector;
    addRequirements(redirector);
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
    double hoodTheta = 40.0048 / Math.pow(distance + 5.88781 , 2) + 0.55959;

    redirector.setTargetPosition(hoodTheta);
  }
}