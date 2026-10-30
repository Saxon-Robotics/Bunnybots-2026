package frc.robot.subsystems.objectivetracker;

import static frc.robot.util.sim.HarvestHavocPantryLevel.CARROT_SIZE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoAlign;
import frc.robot.util.RobotUtil;
import frc.robot.util.sim.HarvestHavocPantryLevel;
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
  private static final double EXTENDED_CARROT_SIZE = CARROT_SIZE + 0.1;

  private static final double PANTRY_LEVEL_TOLERANCE = 0.15;
  private static final double PANTRY_DEPTH_TOLERANCE = 0.3;
  private static final double MAX_OBJ_DISTANCE = 7.5;

  private static final double OBJECT_HEIGHT = 3.0; // placeholder

  private static final int PANTRY_CONFIRMATIONS = 3;
  private static final double PANTRY_CANDIDATE_TIMEOUT = 1.0;

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

  // pieces vision has seen in the pantry but not confirmed yet
  private final List<PantryCandidate> pantryCandidates = new ArrayList<>();

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
    // updatePantryFromVision();

    // RP checks
    if (!isTrackingRP) return;
    if (!stockedUp) stockedUp = checkStockedUp();
  }

  /** Returns the open slot on a pantry level closest to the robot, or null if the level is full. */
  public Pose2d getPantryPose(int level, Pose2d robotPose) {
    double robotX = robotPose.getX();
    double pantryMinX = getPantryCenter().getX() - HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;
    double pantryMaxX = getPantryCenter().getX() + HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;

    List<Double> sorted = pantry.get(level);
    Collections.sort(sorted);

    List<Double> candidates = new ArrayList<>();

    if (sorted.isEmpty()) {
      candidates.add(pantryMinX + EXTENDED_CARROT_SIZE - 0.05);
    } else {
      candidates.add(sorted.get(0) - EXTENDED_CARROT_SIZE);
      candidates.add(sorted.get(sorted.size() - 1) + EXTENDED_CARROT_SIZE);
    }

    for (int i = 0; i < sorted.size() - 1; i++) {
      double left = sorted.get(i);
      double right = sorted.get(i + 1);

      if (right - left >= 2 * EXTENDED_CARROT_SIZE) {
        candidates.add(left + EXTENDED_CARROT_SIZE);
        candidates.add(right - EXTENDED_CARROT_SIZE);
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

    Pose2d pantryPose = AutoAlign.Target.PANTRY.getPose(RobotUtil.isRedAlliance());
    return new Pose2d(bestX, pantryPose.getY(), pantryPose.getRotation());
  }

  private static Translation2d getPantryCenter() {
    return RobotUtil.isRedAlliance()
        ? HarvestHavocPantryLevel.redPantry
        : HarvestHavocPantryLevel.bluePantry;
  }

  /** Adds game pieces that vision sees in our pantry so other people scoring works */
  // private void updatePantryFromVision() {
  //   double now = Timer.getTimestamp();
  //   pantryCandidates.removeIf(candidate -> now - candidate.lastSeen > PANTRY_CANDIDATE_TIMEOUT);

  //   Pose2d robotPose = drive.getPose();
  //   Translation2d pantryCenter = getPantryCenter();
  //   Set<PantryCandidate> seenThisLoop = new HashSet<>();

  //   for (Translation3d piece : vision.getGamePiecePositions(robotPose)) {
  //     if (Math.abs(piece.getY() - pantryCenter.getY()) > PANTRY_DEPTH_TOLERANCE) continue;

  //     double x = piece.getX();
  //     if (Math.abs(x - pantryCenter.getX()) > HarvestHavocPantryLevel.PANTRY_HALF_WIDTH)
  // continue;
  //     if (piece.toTranslation2d().getDistance(robotPose.getTranslation()) > MAX_OBJ_DISTANCE)
  //       continue;

  //     int level = getPantryLevel(piece.getZ());
  //     if (level < 0 || isTrackedInPantry(level, x)) continue;

  //     PantryCandidate candidate = findPantryCandidate(level, x);
  //     if (candidate == null) {
  //       candidate = new PantryCandidate(level, x);
  //       pantryCandidates.add(candidate);
  //     }
  //     if (seenThisLoop.add(candidate)) candidate.addSighting(x, now);
  //     if (candidate.sightings >= PANTRY_CONFIRMATIONS && hasPantrySlot(level)) {
  //       pantry.get(level).add(candidate.x);
  //       pantryCandidates.remove(candidate);
  //     }
  //   }
  // }

  /** Returns the pantry level a piece at this height is sitting on, or -1 if none. */
  private static int getPantryLevel(double z) {
    for (int level = 0; level < HarvestHavocPantryLevel.heights.length; level++) {
      double levelZ = HarvestHavocPantryLevel.heights[level].getZ() + OBJECT_HEIGHT;
      if (Math.abs(z - levelZ) < PANTRY_LEVEL_TOLERANCE) return level;
    }
    return -1;
  }

  private boolean isTrackedInPantry(int level, double x) {
    for (double trackedX : pantry.get(level)) {
      if (Math.abs(trackedX - x) < EXTENDED_CARROT_SIZE / 2) return true;
    }
    return false;
  }

  private PantryCandidate findPantryCandidate(int level, double x) {
    for (PantryCandidate candidate : pantryCandidates) {
      if (candidate.level == level && Math.abs(candidate.x - x) < EXTENDED_CARROT_SIZE / 2) {
        return candidate;
      }
    }
    return null;
  }

  private static class PantryCandidate {
    final int level;
    double x;
    int sightings;
    double lastSeen;

    PantryCandidate(int level, double x) {
      this.level = level;
      this.x = x;
    }

    void addSighting(double newX, double timestamp) {
      x = (x * sightings + newX) / (sightings + 1);
      sightings++;
      lastSeen = timestamp;
    }
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
    return pantry.get(level).size() < 5 && getPantryPose(level, drive.getPose()) != null;
  }
}
