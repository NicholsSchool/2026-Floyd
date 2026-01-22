package frc.robot.subsystems.drive;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Objects;

import com.studica.frc.Navx;

/** Hardware interface for the NAVX 3 axis gyro */
public class GyroIONAVX implements GyroIO {
  private final Navx navx;

  /** Constructor to initialize the NAVX */
  public GyroIONAVX() {
    Constants.RobotType robotType = Objects.requireNonNull(Constants.getRobot());
    switch (robotType) {
      case ROBOT_REAL_FRANKENLEW:
      case ROBOT_REAL:
        navx = new Navx(Constants.DriveConstants.navXPort);
        break;
      default:  
        throw new RuntimeException("Invalid robot for NAVX");
    }
  }

  /**
   * Update the AK hardware inputs
   *
   * @param inputs the inputs to update
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = !navx.getYaw().equals(null);
    // navx uses positive yaw turn to right, so flip sign
    inputs.yawPositionRad = -navx.getYaw().abs(Radian);
    inputs.yawVelocityRadPerSec = -navx.getAngularVel()[3].abs(RadiansPerSecond);

  }

  @Override
  public void resetIMU() {
    System.out.println("resetting imu");
    navx.resetYaw();
  }
}
