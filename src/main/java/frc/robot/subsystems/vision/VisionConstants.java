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
              Filesystem.getDeployDirectory().toPath().resolve("bb-apriltag-layout.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_0_NAME = "Front_Left";
  public static final String CAMERA_1_NAME = "Front_Right";
  public static final String CAMERA_2_NAME = "Back_Left";
  public static final String CAMERA_3_NAME = "Back_Right";

  // Robot to camera transforms
  // (Not used by EagleEye, configure in web UI instead)
  public static Transform3d CAMERA_0_OFFSET =
      new Transform3d(
          Units.inchesToMeters(13.69), // x, forward
          Units.inchesToMeters(0.0), // y, left
          Units.inchesToMeters(13.77), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(-20.0), // pitch
              Units.degreesToRadians(0.0))); // yaw
  public static Transform3d CAMERA_1_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-0.5), // x, forward
          Units.inchesToMeters(13.421), // y, left
          Units.inchesToMeters(7.75), // z, up
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(0.0), // pitch
              Units.degreesToRadians(90))); // yaw
  public static final Transform3d CAMERA_2_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-0.5), // x, forward
          Units.inchesToMeters(-13.421), // y, left
          Units.inchesToMeters(7.75), // z, up
          new Rotation3d(
              Units.degreesToRadians(2.0), // roll
              Units.degreesToRadians(0.0), // pitch
              Units.degreesToRadians(-90))); // yaw
  public static final Transform3d CAMERA_3_OFFSET =
      new Transform3d(
          Units.inchesToMeters(-0.5), // x, forward
          Units.inchesToMeters(-13.421), // y, left
          Units.inchesToMeters(7.75), // z, up
          new Rotation3d(
              Units.degreesToRadians(2.0), // roll
              Units.degreesToRadians(0.0), // pitch
              Units.degreesToRadians(-90))); // yaw

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.175;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.055; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.3; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0, 1.0};

  // Multipliers to apply for MegaTag 2 observations (Limelight only)
  public static final double LINEAR_STD_DEV_MEGATAG_2_FACTOR =
      0.5; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
      Double.POSITIVE_INFINITY; // No rotation data available
}
