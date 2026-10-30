package frc.robot.util;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
  private static final double BUMPER_THICKNESS = 5.5;
  private static final Translation2d ROBOT_OFFSET =
      new Translation2d(
          Units.inchesToMeters(29.5 + BUMPER_THICKNESS) / 2.0,
          Units.inchesToMeters(30.0 + BUMPER_THICKNESS) / 2.0);

  public enum Target {
    NONE(Pose2d.kZero, Pose2d.kZero, Translation2d.kZero),
    PANTRY(
        FieldConstants.BLUE_PANTRY,
        FieldConstants.RED_PANTRY,
        new Translation2d(-ROBOT_OFFSET.getX(), 0)),
    OVEN(
        FieldConstants.BLUE_OVEN,
        FieldConstants.RED_OVEN,
        new Translation2d(ROBOT_OFFSET.getX(), 0)),
    RAMP(
        FieldConstants.BLUE_RAMP,
        FieldConstants.RED_RAMP,
        new Translation2d(-ROBOT_OFFSET.getX(), 0)),
    DEPOT(Pose2d.kZero, Pose2d.kZero, Translation2d.kZero),
    SIDE_DEPOT(
        FieldConstants.BLUE_SIDE_DEPOT,
        FieldConstants.RED_SIDE_DEPOT,
        new Translation2d(ROBOT_OFFSET.getX(), 0)),
    REAR_DEPOT(
        FieldConstants.BLUE_REAR_DEPOT,
        FieldConstants.RED_REAR_DEPOT,
        new Translation2d(ROBOT_OFFSET.getX(), 0)),
    TABLE(
        FieldConstants.BLUE_TABLE_ZONE,
        FieldConstants.RED_TABLE_ZONE,
        new Translation2d(ROBOT_OFFSET.getX(), 0));

    public final Pose2d bluePose;
    public final Pose2d redPose;

    Target(Pose2d bluePose, Pose2d redPose, Translation2d offset) {
      this.bluePose = bluePose.plus(new Transform2d(offset, Rotation2d.kZero));
      this.redPose = redPose.plus(new Transform2d(offset, Rotation2d.kZero));
    }

    public Pose2d getPose(boolean isRedAlliance) {
      return isRedAlliance ? redPose : bluePose;
    }
  }

  private AutoAlign() {
    /* This utility class should not be instantiated */
  }

  @Getter private static Target lastTarget = Target.NONE;
  @Getter private static Pose2d lastTargetPose = Pose2d.kZero;
  @Getter private static APTarget lastAPTarget = new APTarget(lastTargetPose);

  /** Set and retrieve the goal of an {@link AutoAlign.Target}. */
  public static Pose2d setTarget(Target target, Pose2d robotPose) {
    boolean isRedAlliance = RobotUtil.isRedAlliance();
    target = getTarget(target, robotPose, isRedAlliance);

    lastTargetPose = target.getPose(isRedAlliance);
    lastAPTarget = new APTarget(lastTargetPose);
    Logger.recordOutput("AutoAlign/Target", target);
    Logger.recordOutput("AutoAlign/TargetPose", lastTargetPose);
    return lastTargetPose;
  }

  /**
   * Set a specific target pose and relate it to a target enum for logging. If the pose has no
   * correlation, pass in {@link Target#NONE}.
   */
  public static void setTargetPose(Target target, Pose2d targetPose) {
    lastTarget = target;
    lastTargetPose = targetPose;
    lastAPTarget = new APTarget(lastTargetPose);
    Logger.recordOutput("AutoAlign/Target", lastTarget);
    Logger.recordOutput("AutoAlign/TargetPose", lastTargetPose);
  }

  private static Target getTarget(Target target, Pose2d robotPose, boolean isRedAlliance) {
    if (target == Target.DEPOT) {
      if (robotPose
              .getTranslation()
              .getSquaredDistance(Target.SIDE_DEPOT.getPose(isRedAlliance).getTranslation())
          > robotPose
              .getTranslation()
              .getSquaredDistance(Target.REAR_DEPOT.getPose(isRedAlliance).getTranslation())) {
        target = Target.REAR_DEPOT;
      } else {
        target = Target.SIDE_DEPOT;
      }
    }
    lastTarget = target;
    return target;
  }
}
