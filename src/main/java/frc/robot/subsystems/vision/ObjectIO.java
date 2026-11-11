package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectIO {
  @AutoLog
  class ObjectIOInputs {
    public boolean connected;
    public VisionIO.TargetObservation latestTargetObservation =
        new VisionIO.TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public ObjectObservation[] objectObservations = new ObjectObservation[0];
  }

  record ObjectObservation(
      double timestamp,
      Transform3d relativePosition,
      double ambiguity,
      ObjectObservationType type) {}

  enum ObjectObservationType {
    INVALID(false),
    CARROT(true),
    CARROT_CAKE(true),
    FOREIGN_ROBOT(false);

    public final boolean isGamePiece;

    ObjectObservationType(boolean isGamePiece) {
      this.isGamePiece = isGamePiece;
    }
  }

  default void updateInputs(ObjectIOInputs inputs) {}
}
