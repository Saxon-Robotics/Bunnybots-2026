package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public final class VisionConstants {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT;

  static {
    try {
      APRIL_TAG_LAYOUT =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath().resolve("vision/bb-apriltag-layout.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_0_NAME = "Front";
  public static final String CAMERA_1_NAME = "SideLeft";
  public static final String CAMERA_2_NAME = "SideRight";
  public static final String CAMERA_3_NAME = "Rear";

  // Robot to camera transforms
  // (Not used by EagleEye, configure in web UI instead)
  public static Transform3d CAMERA_0_OFFSET =
      new Transform3d(
          Units.inchesToMeters(10.599), // x, forward
          Units.inchesToMeters(6.212), // y, left
          Units.inchesToMeters(7.073), // z, up
          new Rotation3d(
              Units.degreesToRadians(5), // roll
              Units.degreesToRadians(-15), // pitch
              Units.degreesToRadians(-20))); // yaw
  public static Transform3d CAMERA_1_OFFSET =
      new Transform3d(
          Units.inchesToMeters(3.057), // x, forward
          Units.inchesToMeters(15.186), // y, left
          Units.inchesToMeters(9.885), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(0.0), // pitch
              Units.degreesToRadians(50))); // yaw
  public static final Transform3d CAMERA_2_OFFSET =
      new Transform3d(
          Units.inchesToMeters(3.128), // x, forward
          Units.inchesToMeters(-15.101), // y, left
          Units.inchesToMeters(9.772), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(0.0), // pitch
              Units.degreesToRadians(-50))); // yaw
  public static final Transform3d CAMERA_3_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-14.095), // x, forward
          Units.inchesToMeters(-11.788), // y, left
          Units.inchesToMeters(7.617), // z, up
          new Rotation3d(
              Units.degreesToRadians(-10), // roll
              Units.degreesToRadians(-22.5), // pitch
              Units.degreesToRadians(180.0 - 35.0))); // yaw
  public static final Transform3d CAMERA_4_BASE_OFFSET =
      new Transform3d(
          Units.inchesToMeters(0), // x, forward
          Units.inchesToMeters(0), // y, left
          Units.inchesToMeters(0), // z, elevator stowed height
          new Rotation3d(
              Units.degreesToRadians(0), // roll
              Units.degreesToRadians(30), // pitch
              Units.degreesToRadians(0))); // yaw

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.175;
  public static final double MAX_Z_ERROR = 0.75;
  public static final double MAX_OBJ_AMBIGUITY = 0.3;

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.055; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.3; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STD_DEV_FACTORS = {1.0, 1.0, 1.0, 1.0};

  // Multipliers to apply for MegaTag 2 observations (Limelight only)
  public static final double LINEAR_STD_DEV_MEGATAG_2_FACTOR =
      0.5; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
      Double.POSITIVE_INFINITY; // No rotation data available
}
