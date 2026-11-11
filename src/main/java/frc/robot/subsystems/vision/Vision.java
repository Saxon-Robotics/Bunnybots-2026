// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTracer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final ObjectIO[] objIO;
  private final ObjectIOInputsAutoLogged[] objInputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, io, new ObjectIO[0]);
  }

  public Vision(VisionConsumer consumer, VisionIO[] io, ObjectIO[] objIO) {
    this.consumer = consumer;
    this.io = io;
    this.objIO = objIO;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
    this.objInputs = new ObjectIOInputsAutoLogged[objIO.length];
    for (int i = 0; i < objInputs.length; i++) {
      objInputs[i] = new ObjectIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length + objIO.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
    for (int i = 0; i < objInputs.length; i++) {
      disconnectedAlerts[i + io.length] =
          new Alert("Object detection camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    for (int i = 0; i < objIO.length; i++) {
      objIO[i].updateInputs(objInputs[i]);
      Logger.processInputs("Vision/ObjectDetection/Camera" + i, objInputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= LINEAR_STD_DEV_MEGATAG_2_FACTOR;
          angularStdDev *= ANGULAR_STD_DEV_MEGATAG_2_FACTOR;
        }
        if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
          linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
          angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    LoggedTracer.record("Vision/AprilTag");
  }

  public List<Pair<ObjectIO.ObjectObservationType, Pose3d>> getGamePiecePoses(Pose2d robotPose) {
    // Calculate max possible capacity
    int capacity = 0;
    for (var input : objInputs) {
      capacity += input.objectObservations.length;
    }

    List<Pair<ObjectIO.ObjectObservationType, Pose3d>> gamePiecePosesAccepted =
        new ArrayList<>(capacity);
    List<Pose3d> carrotPosesAccepted = new ArrayList<>(capacity);
    List<Pose3d> cakePosesAccepted = new ArrayList<>(capacity);
    List<Pose3d> gamePiecePosesRejected = new ArrayList<>(capacity);

    // Combine all foreign pose estimates
    Pose3d robotPose3d = new Pose3d(robotPose);
    for (var input : objInputs) {
      for (var object : input.objectObservations) {
        if (!object.type().isGamePiece) continue;
        if (object.ambiguity() <= MAX_OBJ_AMBIGUITY) {
          ObjectIO.ObjectObservationType objectType = object.type();
          // Transform from robot relative to global poses
          Pose3d objectPose = robotPose3d.plus(object.relativePosition());
          gamePiecePosesAccepted.add(Pair.of(objectType, objectPose));
          if (objectType == ObjectIO.ObjectObservationType.CARROT) {
            carrotPosesAccepted.add(objectPose);
          } else {
            cakePosesAccepted.add(objectPose);
          }
        } else {
          gamePiecePosesRejected.add(robotPose3d.plus(object.relativePosition()));
        }
      }
    }

    Logger.recordOutput(
        "Vision/ObjectDetection/CarrotPosesAccepted", carrotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/CarrotCakePosesAccepted", cakePosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/GamePiecePosesRejected",
        gamePiecePosesRejected.toArray(new Pose3d[0]));

    return gamePiecePosesAccepted;
  }

  public Pose2d[] getForeignRobotPoses(Pose2d robotPose) {
    // Calculate max possible capacity
    int capacity = 0;
    for (var input : objInputs) {
      capacity += input.objectObservations.length;
    }

    List<Pose2d> foreignRobotPosesAccepted = new ArrayList<>(capacity);
    List<Pose2d> foreignRobotPosesRejected = new ArrayList<>(capacity);

    // Combine all foreign pose estimates
    Pose3d robotPose3d = new Pose3d(robotPose);
    for (var input : objInputs) {
      for (var object : input.objectObservations) {
        if (object.type() != ObjectIO.ObjectObservationType.FOREIGN_ROBOT) continue;
        if (object.ambiguity() <= MAX_OBJ_AMBIGUITY) {
          // Transform from robot relative to global poses
          foreignRobotPosesAccepted.add(robotPose3d.plus(object.relativePosition()).toPose2d());
        } else {
          foreignRobotPosesRejected.add(robotPose3d.plus(object.relativePosition()).toPose2d());
        }
      }
    }

    Logger.recordOutput(
        "Vision/ObjectDetection/ForeignRobotPosesAccepted",
        foreignRobotPosesAccepted.toArray(new Pose2d[0]));
    Logger.recordOutput(
        "Vision/ObjectDetection/ForeignRobotPosesRejected",
        foreignRobotPosesRejected.toArray(new Pose2d[0]));

    return foreignRobotPosesAccepted.toArray(new Pose2d[0]);
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
