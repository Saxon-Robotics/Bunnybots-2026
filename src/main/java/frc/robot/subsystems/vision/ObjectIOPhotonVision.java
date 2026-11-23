package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class ObjectIOPhotonVision implements ObjectIO {
  private final PhotonCamera camera;
  private final Supplier<Transform3d> robotToCamera;

  /**
   * Creates a new ObjectIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera A supplier of the current 3D position of the camera relative to the robot.
   */
  public ObjectIOPhotonVision(String name, Supplier<Transform3d> robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  /**
   * Creates a new ObjectIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public ObjectIOPhotonVision(String name, Transform3d robotToCamera) {
    this(name, () -> robotToCamera);
  }

  @Override
  public void updateInputs(ObjectIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    List<ObjectObservation> objectObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new VisionIO.TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation =
            new VisionIO.TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // Update object observations
      for (var target : result.targets) {
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d robotToTarget = robotToCamera.get().plus(cameraToTarget);
        objectObservations.add(
            new ObjectObservation(
                result.getTimestampSeconds(),
                robotToTarget,
                target.poseAmbiguity,
                switch (target.objDetectId) {
                  case 0 -> ObjectObservationType.CARROT;
                  case 1 -> ObjectObservationType.CARROT_CAKE;
                  case 2 -> ObjectObservationType.FOREIGN_ROBOT;
                  default -> ObjectObservationType.INVALID;
                }));
      }
    }

    // Save observations to inputs object
    inputs.objectObservations = objectObservations.toArray(new ObjectObservation[0]);
  }
}
