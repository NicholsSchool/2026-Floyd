package frc.robot.commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.redirector.Redirector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.AllianceFlipUtil;

public class ShootOnMove extends InstantCommand {
  private Drive drive;
  private Shooter shooter;
  private Redirector redirector;
  private Turret turret;

  public ShootOnMove(Drive drive, Shooter shooter, Redirector redirector, Turret turret) {
    this.drive = drive;
    this.shooter = shooter;
    this.redirector = redirector;
    this.turret = turret;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    var fieldVelocity = drive.getFieldVelocity();
    var currentPose = new Pose2d(new Translation2d(drive.getTurretPose().getX() + fieldVelocity.dx * AutoConstants.SHOOT_ON_MOVE_FUTURE_MULTIPLIER,
     drive.getTurretPose().getY() + fieldVelocity.dy * AutoConstants.SHOOT_ON_MOVE_FUTURE_MULTIPLIER),
     drive.getTurretPose().getRotation());
     
    Translation2d hubOffset = (AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())).minus(currentPose.getTranslation());
    double distance = hubOffset.getNorm() + AutoConstants.AUTOAIM_GASLIGHT;

    double shooterMetersPerSec = -10.38954 * Math.sin(1.66397 * Math.pow(distance - 0.621899, 0.2)) + 18.42387;
    double hoodTheta = (4.24803 / Math.pow(distance + 3.24144 , 2) + 1.23079);
    double turretAngle = Math.atan2(hubOffset.getY(), hubOffset.getX()) - currentPose.getRotation().getRadians();

    Translation3d autoaimVector = new Translation3d(shooterMetersPerSec * Math.cos(hoodTheta) * Math.cos(turretAngle),
     shooterMetersPerSec * Math.cos(hoodTheta) * Math.sin(turretAngle), shooterMetersPerSec * Math.sin(hoodTheta));

    Translation3d shootOnMoveVector = new Translation3d(autoaimVector.getX() - fieldVelocity.dx, autoaimVector.getY() - fieldVelocity.dy, autoaimVector.getZ());

    double turretMovingAngle = Math.atan2(shootOnMoveVector.getY(), shootOnMoveVector.getX());

    if(turretMovingAngle < TurretConstants.TURRET_MIN_ANGLE + TurretConstants.TURRET_SOFT_LIMIT){
        turretMovingAngle = TurretConstants.TURRET_MAX_ANGLE - TurretConstants.TURRET_SOFT_LIMIT;
    }else if(turretMovingAngle > TurretConstants.TURRET_MAX_ANGLE - TurretConstants.TURRET_SOFT_LIMIT){
        turretMovingAngle = TurretConstants.TURRET_MIN_ANGLE + TurretConstants.TURRET_SOFT_LIMIT;
    }

    double redirectorMovingAngle = (Math.PI / 2) - Math.atan(Math.hypot(shootOnMoveVector.getY(), shootOnMoveVector.getX()) / shootOnMoveVector.getZ());
    double movingVelocity = Math.sqrt(Math.pow(shootOnMoveVector.getX(), 2) + Math.pow(shootOnMoveVector.getY(), 2) + Math.pow(shootOnMoveVector.getZ(), 2));

    turret.setTargetPosition(turretMovingAngle);
    redirector.setTargetPosition(redirectorMovingAngle);
    shooter.setVelMPS(movingVelocity, redirectorMovingAngle);

    turret.setTargetPosition(turretAngle);
    redirector.setTargetPosition(hoodTheta);
    shooter.setVelMPS(shooterMetersPerSec, hoodTheta);


  }
}
