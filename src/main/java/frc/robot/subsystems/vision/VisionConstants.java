// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.defaultAprilTagType.getLayout();

  // Camera names, must match names configured on coprocessor
   public static String camera0Name = "camUp";
  public static Transform3d robotToCamera0 =
  new Transform3d(0.10, -0.01, 0.407, new Rotation3d(Math.toRadians(-1.0), Math.toRadians(-28.0), Math.toRadians(0.0)));
  //13 y 9 3/4 x
  //pitch is normally 35.0 degrees, changed for now cause it's not working
  //-3 degree yaw offset

  public static String camera1Name = "camL";
  public static Transform3d robotToCamera1 =
  new Transform3d(-0.28, 0.304, 0.185, new Rotation3d(0.0, 0.0, Math.PI / 2));
//   public static String camera1Name = "Arducam_OV2311_USB_Camera-B";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead
//   // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.03; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        3.0 // Camera 1 
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

    public static final int visionStatsNumBuffer = 100;
}
