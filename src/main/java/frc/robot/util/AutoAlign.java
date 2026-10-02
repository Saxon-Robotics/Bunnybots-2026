package frc.robot.util;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {
  private static final double BUMPER_THICKNESS = 4;
  private static final Translation2d ROBOT_OFFSET =
      new Translation2d(
          Units.inchesToMeters(29.5 + BUMPER_THICKNESS) / 2.0,
          Units.inchesToMeters(30.0 + BUMPER_THICKNESS) / 2.0);

  public enum Target {
    PANTRY(
        FieldConstants.BLUE_PANTRY,
        FieldConstants.RED_PANTRY,
        new Translation2d(0, -ROBOT_OFFSET.getY())),
    OVEN(
        FieldConstants.BLUE_OVEN,
        FieldConstants.RED_OVEN,
        new Translation2d(0, ROBOT_OFFSET.getY())),
    RAMP(
        FieldConstants.BLUE_RAMP,
        FieldConstants.RED_RAMP,
        new Translation2d(ROBOT_OFFSET.getX(), 0)),
    DEPOT(Pose2d.kZero, Pose2d.kZero, Translation2d.kZero),
    SIDE_DEPOT(
        FieldConstants.BLUE_SIDE_DEPOT,
        FieldConstants.RED_SIDE_DEPOT,
        new Translation2d(0, ROBOT_OFFSET.getY())),
    REAR_DEPOT(
        FieldConstants.BLUE_REAR_DEPOT,
        FieldConstants.RED_REAR_DEPOT,
        new Translation2d(-ROBOT_OFFSET.getX(), 0)),
    TABLE(FieldConstants.BLUE_TABLE_ZONE, FieldConstants.RED_TABLE_ZONE, ROBOT_OFFSET);

    public final Pose2d bluePose;
    public final Pose2d redPose;

    Target(Pose2d bluePose, Pose2d redPose, Translation2d offset) {
      this.bluePose = bluePose.plus(new Transform2d(offset, Rotation2d.kZero));
      this.redPose = redPose.plus(new Transform2d(offset.times(-1), Rotation2d.kZero));
    }

    public Pose2d getPose(boolean isRedAlliance) {
      return isRedAlliance ? redPose : bluePose;
    }
  }

  private AutoAlign() {
    /* This utility class should not be instantiated */
  }

  @Getter private static Pose2d lastTarget = Pose2d.kZero;
  @Getter private static APTarget lastAPTarget = new APTarget(lastTarget);

  public static void setTargetPose(Target target, Pose2d robotPose) {
    boolean isRedAlliance = RobotUtil.isRedAlliance();
    target = getTarget(target, robotPose, isRedAlliance);

    lastTarget = target.getPose(isRedAlliance);
    lastAPTarget = new APTarget(lastTarget);
    Logger.recordOutput("AutoAlign/Target", target);
    Logger.recordOutput("AutoAlign/TargetPose", lastTarget);
  }

  public static Target getTarget(Target target, Pose2d robotPose, boolean isRedAlliance) {
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
    return target;
  }
}
