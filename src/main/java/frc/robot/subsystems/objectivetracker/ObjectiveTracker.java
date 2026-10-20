package frc.robot.subsystems.objectivetracker;

import static frc.robot.util.sim.HarvestHavocPantryLevel.CARROT_SIZE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoAlign;
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import lombok.Getter;

public class ObjectiveTracker extends SubsystemBase {
  public enum GamePiece {
    CARROT,
    CARROT_CAKE
  }

  private static final double NEAR_THRESHOLD = 0.6;
  private static final double PANTRY_MIN_X_BLUE =
      Constants.FieldConstants.ORIGIN.getX()
          - 7.928
          - 1.194; // just realized we have to take into account alliances welp
  private static final double PANTRY_MAX_X_BLUE =
      Constants.FieldConstants.ORIGIN.getX() - 7.928 + 1.194;
  private static final double PANTRY_MIN_X_RED =
      Constants.FieldConstants.ORIGIN.getX()
          + 5.54
          - 1.194; // just realized we have to take into account alliances welp
  private static final double PANTRY_MAX_X_RED =
      Constants.FieldConstants.ORIGIN.getX() + 5.54 + 1.194;

  private static ObjectiveTracker instance;

  public static ObjectiveTracker getInstance() {
    if (instance == null) {
      throw new IllegalStateException("ObjectiveTracker has not been initialized");
    }
    return instance;
  }

  public static ObjectiveTracker initialize(
      Drive drive, Vision vision, Elevator elevator, Gripper gripper, Trader trader) {
    instance = new ObjectiveTracker(drive, vision, elevator, gripper, trader);
    return instance;
  }

  // tracked subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Gripper gripper;
  private final Trader trader;

  // not gonna track number of pieces because that's annoying
  @Getter private GamePiece traderHeld;
  @Getter private GamePiece gripperHeld;

  // x positions of each game piece in pantry
  @Getter private final List<List<Double>> pantry = new ArrayList<>(3);

  // whether we've dumped carrots in oven
  @Getter private boolean cakeReady;

  // RP tracking
  public final boolean isTrackingRP;
  private int bakedUpProgress;
  @Getter private boolean stockedUp;

  private ObjectiveTracker(
      Drive drive, Vision vision, Elevator elevator, Gripper gripper, Trader trader) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.gripper = gripper;
    this.trader = trader;

    for (int i = 0; i < 3; i++) {
      pantry.add(new ArrayList<>(5));
    }

    // set up triggers
    new Trigger(gripper::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (AutoAlign.getLastTarget() == AutoAlign.Target.RAMP) {
                    gripperHeld = GamePiece.CARROT_CAKE;
                    cakeReady = false;
                  } else {
                    gripperHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  int scoringIndex = elevator.getSetpoint().scoringIndex;
                  if (scoringIndex > -1 && AutoAlign.getLastTarget() == AutoAlign.Target.PANTRY) {
                    pantry.get(scoringIndex).add(drive.getPose().getX());
                  }
                  gripperHeld = null;
                }));

    new Trigger(trader::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (AutoAlign.getLastTarget() == AutoAlign.Target.RAMP) {
                    traderHeld = GamePiece.CARROT_CAKE;
                  } else {
                    traderHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (drive
                          .getPose()
                          .getTranslation()
                          .getSquaredDistance(
                              AutoAlign.Target.OVEN
                                  .getPose(RobotUtil.isRedAlliance())
                                  .getTranslation())
                      < NEAR_THRESHOLD) {
                    traderHeld = null;
                    cakeReady = true;
                  }
                }));

    isTrackingRP = DriverStation.getMatchType() == DriverStation.MatchType.Qualification;
  }

  @Override
  public void periodic() {
    // RP checks
    if (!isTrackingRP) return;
    if (!stockedUp) stockedUp = checkStockedUp();
  }

  /** Returns the open slot on a pantry level closest to the robot, or null if the level is full. */
  public Pose2d getPantryPose(int level, Pose2d robotPose) {
    double robotX = robotPose.getX();
    double pantryMinX;
    double pantryMaxX;

    if (RobotUtil.isRedAlliance()) {
      pantryMinX = PANTRY_MIN_X_RED;
      pantryMaxX = PANTRY_MAX_X_RED;
    } else {
      pantryMinX = PANTRY_MIN_X_BLUE;
      pantryMaxX = PANTRY_MAX_X_BLUE;
    }

    List<Double> sorted = pantry.get(level);
    Collections.sort(sorted);

    List<Double> candidates = new ArrayList<>();

    if (sorted.isEmpty()) {
      candidates.add(pantryMinX + CARROT_SIZE);
    } else {
      candidates.add(sorted.get(0) - CARROT_SIZE);
      candidates.add(sorted.get(sorted.size() - 1) + CARROT_SIZE);
    }

    for (int i = 0; i < sorted.size() - 1; i++) {
      double left = sorted.get(i);
      double right = sorted.get(i + 1);

      if (right - left >= 2 * CARROT_SIZE) {
        candidates.add(left + CARROT_SIZE);
        candidates.add(right - CARROT_SIZE);
      }
    }

    Double bestX = null;
    for (double x : candidates) {
      if (x < pantryMinX || x > pantryMaxX) {
        continue;
      }
      if (bestX == null || Math.abs(x - robotX) < Math.abs(bestX - robotX)) {
        bestX = x;
      }
    }

    if (bestX == null) return null;

    Pose2d pantry = AutoAlign.Target.PANTRY.getPose(RobotUtil.isRedAlliance());
    return new Pose2d(bestX, pantry.getY(), pantry.getRotation());
  }

  public boolean checkStockedUp() {
    for (List<Double> level : pantry) {
      if (level.size() < 3) return false;
    }
    return true;
  }

  public boolean isBakedUp() {
    return bakedUpProgress > 2;
  }

  public boolean hasPantrySlot(int level) {
    return pantry.get(level).size() < 5;
  }
}
