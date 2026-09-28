package frc.robot.subsystems.vision;

import frc.robot.util.io.vision.EagleEyeCamera;
import java.util.List;

/** IO implementation for real EagleEye hardware. */
public class VisionIOEagleEye implements VisionIO {
  private final EagleEyeCamera camera;

  /**
   * Creates a new VisionIOEagleEye.
   *
   * @param key The target key of the camera in NetworkTables.
   */
  public VisionIOEagleEye(String key) {
    camera =
        new EagleEyeCamera(
            "heartbeat/" + key + "/time",
            "localization/" + key + "/pose",
            "localization/" + key + "/meta");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<PoseObservation> poseObservations = camera.poll();
    inputs.connected = camera.isConnected();
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.isTagVisible = !poseObservations.isEmpty();
  }
}
