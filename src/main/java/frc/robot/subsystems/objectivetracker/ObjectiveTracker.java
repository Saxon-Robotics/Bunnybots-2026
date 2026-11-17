package frc.robot.subsystems.objectivetracker;

import static frc.robot.util.sim.HarvestHavocPantryLevel.CARROT_SIZE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.ObjectIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoAlign;
import frc.robot.util.RobotUtil;
import frc.robot.util.sim.HarvestHavocPantryLevel;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ObjectiveTracker extends SubsystemBase {
  public enum GamePiece {
    CARROT,
    CARROT_CAKE
  }

  private static final double NEAR_THRESHOLD = 0.6;
  // how far past the pantry's ends and off the scoring line a gripper release still counts
  private static final double PANTRY_SCORE_X_MARGIN = 0.3;
  private static final double PANTRY_SCORE_Y_TOLERANCE = 0.75;
  private static final double EXTENDED_CARROT_SIZE = CARROT_SIZE + 0.1;

  private static final double PANTRY_LEVEL_TOLERANCE = 0.15;
  private static final double PANTRY_DEPTH_TOLERANCE = 0.3;
  private static final double MAX_OBJ_DISTANCE = 7.5;

  private static final double OBJECT_HEIGHT = 3.0; // placeholder

  private static final int PANTRY_CONFIRMATIONS = 3;

  public static final int PANTRY_LEVEL_CAPACITY = 5;

  public static final int STOCKED_UP_PER_LEVEL = 3;

  public static final int CARROTS_PER_CAKE = 3;

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

  @Getter private GamePiece gripperHeld;

  // x positions of each game piece in pantry
  @Getter private final List<List<Double>> pantry = new ArrayList<>(3);

  // pieces vision has seen in the pantry but not confirmed yet
  private final List<PantryCandidate> pantryCandidates = new ArrayList<>();

  private final int[] pantryCakes = new int[3];

  @Getter private int ovenCarrots;
  @Getter private int cakesCollected;

  @Getter private int ovenCakes;

  private final Queue<GamePiece> traderPieces = new ArrayDeque<>(3);
  private static final double TRADER_COUNT_DEBOUNCE_SEC = 0.1;
  private int pendingTraderCount;
  private double pendingTraderCountSince;

  private int heldScoringIndex;

  private final int[] freeSlots = new int[3];
  private final int[] pantryLevelCounts = new int[3];

  @Getter @Setter private boolean collectingCake;

  private boolean freeSlotsForRed;

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
    updateFreeSlots();

    // set up triggers
    new Trigger(gripper::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  gripperHeld = currentPickupType();
                  if (gripperHeld == GamePiece.CARROT_CAKE) cakesCollected++;
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  int scoringIndex = heldScoringIndex;
                  if (scoringIndex > 0 && isNearPantry()) {
                    addPantryPiece(scoringIndex - 1, drive.getPose().getX());
                    if (gripperHeld == GamePiece.CARROT_CAKE) pantryCakes[scoringIndex - 1]++;
                  }
                  gripperHeld = null;
                  heldScoringIndex = 0;
                }));
  }

  /**
   * What a piece picked up right now is assumed to be: a cake if the robot is closer to the ramp
   * than to either depot, otherwise a carrot.
   */
  private GamePiece currentPickupType() {
    if (collectingCake) return GamePiece.CARROT_CAKE;
    boolean isRed = RobotUtil.isRedAlliance();
    Translation2d robot = drive.getPose().getTranslation();
    double rampDist =
        robot.getSquaredDistance(AutoAlign.Target.RAMP.getPose(isRed).getTranslation());
    double depotDist =
        Math.min(
            robot.getSquaredDistance(AutoAlign.Target.SIDE_DEPOT.getPose(isRed).getTranslation()),
            robot.getSquaredDistance(AutoAlign.Target.REAR_DEPOT.getPose(isRed).getTranslation()));
    return rampDist < depotDist ? GamePiece.CARROT_CAKE : GamePiece.CARROT;
  }

  @Override
  public void periodic() {
    // uncomment this when ready
    //    updatePantryFromVision();

    if (RobotUtil.isRedAlliance() != freeSlotsForRed) updateFreeSlots();

    if (gripper.isLoaded()) {
      heldScoringIndex = elevator.getSetpoint().scoringIndex;
    }

    updateTraderPieces();

    Logger.recordOutput(
        "ObjectiveTracker/TraderPieces",
        traderPieces.stream().map(GamePiece::name).toArray(String[]::new));
    Logger.recordOutput("ObjectiveTracker/OvenCarrots", ovenCarrots);
    Logger.recordOutput("ObjectiveTracker/OvenCakes", ovenCakes);
    Logger.recordOutput("ObjectiveTracker/CakesCollected", cakesCollected);
    Logger.recordOutput("ObjectiveTracker/CakeReady", isCakeReady());
    for (int level = 0; level < 3; level++) {
      pantryLevelCounts[level] = getPantryCount(level);
    }
    Logger.recordOutput("ObjectiveTracker/PantryLevelCounts", pantryLevelCounts);
    Logger.recordOutput("ObjectiveTracker/PantryCakes", pantryCakes);
    Logger.recordOutput("ObjectiveTracker/FreeSlots", freeSlots);
    Logger.recordOutput("ObjectiveTracker/StockedUp", isStockedUp());
    Logger.recordOutput("ObjectiveTracker/BakedUp", isBakedUp());
  }

  public void reset() {
    for (List<Double> level : pantry) level.clear();
    pantryCandidates.clear();
    Arrays.fill(pantryCakes, 0);
    ovenCarrots = 0;
    cakesCollected = 0;
    ovenCakes = 0;
    heldScoringIndex = 0;
    collectingCake = false;
    updateFreeSlots();

    traderPieces.clear();
    pendingTraderCount = trader.getNumGamePieces();
    pendingTraderCountSince = Timer.getTimestamp();
    for (int i = 0; i < pendingTraderCount; i++) {
      traderPieces.add(GamePiece.CARROT);
    }
    gripperHeld = gripper.isLoaded() ? GamePiece.CARROT : null;
  }

  private void updateTraderPieces() {
    double now = Timer.getTimestamp();
    int rawCount = trader.getNumGamePieces();
    if (rawCount != pendingTraderCount) {
      pendingTraderCount = rawCount;
      pendingTraderCountSince = now;
    }
    if (now - pendingTraderCountSince < TRADER_COUNT_DEBOUNCE_SEC) return;

    int diff = pendingTraderCount - traderPieces.size();
    if (diff < 0) {
      boolean nearOven = isNearOven();
      for (int i = 0; i < -diff; i++) {
        GamePiece piece = traderPieces.remove();
        if (!nearOven) continue;
        if (piece == GamePiece.CARROT_CAKE) {
          ovenCakes++;
        } else {
          ovenCarrots++;
        }
      }
    } else if (diff > 0) {
      GamePiece type = currentPickupType();
      for (int i = 0; i < diff; i++) {
        traderPieces.add(type);
        if (type == GamePiece.CARROT_CAKE) cakesCollected++;
      }
    }
  }

  private boolean isNearOven() {
    return drive
            .getPose()
            .getTranslation()
            .getSquaredDistance(
                AutoAlign.Target.OVEN.getPose(RobotUtil.isRedAlliance()).getTranslation())
        < NEAR_THRESHOLD;
  }

  private boolean isNearPantry() {
    Translation2d robot = drive.getPose().getTranslation();
    double scoringY = AutoAlign.Target.PANTRY.getPose(RobotUtil.isRedAlliance()).getY();
    return Math.abs(robot.getX() - getPantryCenter().getX())
            < HarvestHavocPantryLevel.PANTRY_HALF_WIDTH + PANTRY_SCORE_X_MARGIN
        && Math.abs(robot.getY() - scoringY) < PANTRY_SCORE_Y_TOLERANCE;
  }

  public boolean isTrackingRP() {
    return DriverStation.getMatchType() == DriverStation.MatchType.Qualification;
  }

  public boolean isCakeReady() {
    return ovenCarrots / CARROTS_PER_CAKE + ovenCakes > cakesCollected;
  }

  public void forfeitCake() {
    if (isCakeReady()) cakesCollected++;
  }

  /** Returns the open slot on a pantry level closest to the robot, or null if the level is full. */
  public Pose2d getPantryPose(int level, Pose2d robotPose) {
    double robotX = robotPose.getX();
    double pantryCenterX = getPantryCenter().getX();
    double pantryMinX = pantryCenterX - HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;
    double pantryMaxX = pantryCenterX + HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;

    List<Double> sorted = pantry.get(level);

    double bestX = Double.NaN;
    if (sorted.isEmpty()) {
      bestX =
          pickCloser(
              bestX, pantryMinX + EXTENDED_CARROT_SIZE - 0.05, robotX, pantryMinX, pantryMaxX);
    } else {
      bestX =
          pickCloser(bestX, sorted.get(0) - EXTENDED_CARROT_SIZE, robotX, pantryMinX, pantryMaxX);
      bestX =
          pickCloser(
              bestX,
              sorted.get(sorted.size() - 1) + EXTENDED_CARROT_SIZE,
              robotX,
              pantryMinX,
              pantryMaxX);
    }

    for (int i = 0; i < sorted.size() - 1; i++) {
      double left = sorted.get(i);
      double right = sorted.get(i + 1);

      if (right - left >= 2 * EXTENDED_CARROT_SIZE) {
        bestX = pickCloser(bestX, left + EXTENDED_CARROT_SIZE, robotX, pantryMinX, pantryMaxX);
        bestX = pickCloser(bestX, right - EXTENDED_CARROT_SIZE, robotX, pantryMinX, pantryMaxX);
      }
    }

    if (Double.isNaN(bestX)) return null;

    Pose2d pantryPose = AutoAlign.Target.PANTRY.getPose(RobotUtil.isRedAlliance());
    return new Pose2d(bestX, pantryPose.getY(), pantryPose.getRotation());
  }

  private static double pickCloser(
      double bestX, double x, double robotX, double pantryMinX, double pantryMaxX) {
    if (x < pantryMinX || x > pantryMaxX) return bestX;
    if (Double.isNaN(bestX) || Math.abs(x - robotX) < Math.abs(bestX - robotX)) return x;
    return bestX;
  }

  private void addPantryPiece(int level, double x) {
    double pantryCenterX = getPantryCenter().getX();
    List<Double> pieces = pantry.get(level);
    pieces.add(
        MathUtil.clamp(
            x,
            pantryCenterX - HarvestHavocPantryLevel.PANTRY_HALF_WIDTH,
            pantryCenterX + HarvestHavocPantryLevel.PANTRY_HALF_WIDTH));
    Collections.sort(pieces);
    updateFreeSlots();
  }

  private void updateFreeSlots() {
    freeSlotsForRed = RobotUtil.isRedAlliance();
    double pantryCenterX = getPantryCenter().getX();
    double pantryMinX = pantryCenterX - HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;
    double pantryMaxX = pantryCenterX + HarvestHavocPantryLevel.PANTRY_HALF_WIDTH;
    for (int level = 0; level < 3; level++) {
      List<Double> sorted = pantry.get(level);
      int slots;
      if (sorted.isEmpty()) {
        double first = pantryMinX + EXTENDED_CARROT_SIZE - 0.05;
        slots = first > pantryMaxX ? 0 : (int) ((pantryMaxX - first) / EXTENDED_CARROT_SIZE) + 1;
      } else {
        slots = (int) ((sorted.get(0) - pantryMinX) / EXTENDED_CARROT_SIZE);
        slots += (int) ((pantryMaxX - sorted.get(sorted.size() - 1)) / EXTENDED_CARROT_SIZE);
        for (int i = 0; i < sorted.size() - 1; i++) {
          slots +=
              Math.max(0, (int) ((sorted.get(i + 1) - sorted.get(i)) / EXTENDED_CARROT_SIZE) - 1);
        }
      }
      freeSlots[level] = Math.max(0, Math.min(slots, PANTRY_LEVEL_CAPACITY - sorted.size()));
    }
  }

  private static Translation2d getPantryCenter() {
    return RobotUtil.isRedAlliance()
        ? HarvestHavocPantryLevel.redPantry
        : HarvestHavocPantryLevel.bluePantry;
  }

  /** Adds game pieces that vision sees in our pantry so other people scoring works */
  private void updatePantryFromVision() {
    double now = Timer.getTimestamp();
    pantryCandidates.removeIf(candidate -> now - candidate.lastSeen > PANTRY_CANDIDATE_TIMEOUT);

    Pose2d robotPose = drive.getPose();
    Translation2d pantryCenter = getPantryCenter();
    Set<PantryCandidate> seenThisLoop = new HashSet<>();

    for (Pair<ObjectIO.ObjectObservationType, Pose3d> piece : vision.getGamePiecePoses(robotPose)) {
      ObjectIO.ObjectObservationType pieceType = piece.getFirst();
      Pose3d piecePose = piece.getSecond();
      if (Math.abs(piecePose.getY() - pantryCenter.getY()) > PANTRY_DEPTH_TOLERANCE) continue;

      double x = piecePose.getX();
      if (Math.abs(x - pantryCenter.getX()) > HarvestHavocPantryLevel.PANTRY_HALF_WIDTH
          || piecePose.getTranslation().toTranslation2d().getDistance(robotPose.getTranslation())
              > MAX_OBJ_DISTANCE) continue;

      int level = getPantryLevel(piecePose.getZ());
      if (level < 0 || isTrackedInPantry(level, x)) continue;

      PantryCandidate candidate = findPantryCandidate(level, x);
      if (candidate == null) {
        candidate = new PantryCandidate(level, x, pieceType);
        pantryCandidates.add(candidate);
      }
      if (seenThisLoop.add(candidate)) candidate.addSighting(x, now);
      if (candidate.sightings >= PANTRY_CONFIRMATIONS && hasPantrySlot(level)) {
        addPantryPiece(level, candidate.x);
        if (candidate.isCarrotCake) pantryCakes[candidate.level] += 1;
        pantryCandidates.remove(candidate);
      }
    }
  }

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
    final boolean isCarrotCake;
    double x;
    int sightings;
    double lastSeen;

    PantryCandidate(int level, double x, ObjectIO.ObjectObservationType pieceType) {
      this.level = level;
      this.x = x;
      this.isCarrotCake = pieceType == ObjectIO.ObjectObservationType.CARROT_CAKE;
    }

    void addSighting(double newX, double timestamp) {
      x = (x * sightings + newX) / (sightings + 1);
      sightings++;
      lastSeen = timestamp;
    }
  }

  public int getPantryCount(int level) {
    return pantry.get(level).size();
  }

  public boolean levelHasCake(int level) {
    return pantryCakes[level] > 0;
  }

  public boolean isStockedUp() {
    for (int level = 0; level < pantry.size(); level++) {
      if (getPantryCount(level) < STOCKED_UP_PER_LEVEL) return false;
    }
    return true;
  }

  public boolean isBakedUp() {
    for (int level = 0; level < pantry.size(); level++) {
      if (!levelHasCake(level)) return false;
    }
    return true;
  }

  public int getFreeSlots(int level) {
    return freeSlots[level];
  }

  public boolean hasPantrySlot(int level) {
    return freeSlots[level] > 0;
  }

  public boolean hasAnyPantrySlot() {
    return hasPantrySlot(0) || hasPantrySlot(1) || hasPantrySlot(2);
  }

  public boolean levelNeedsCake(int level) {
    return !levelHasCake(level) && hasPantrySlot(level);
  }

  public boolean levelNeedsStock(int level) {
    return getPantryCount(level) < STOCKED_UP_PER_LEVEL && hasPantrySlot(level);
  }

  public boolean levelOpenForCarrot(int level) {
    if (levelHasCake(level)) return hasPantrySlot(level);
    return freeSlots[level] > 1;
  }
}
