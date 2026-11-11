package frc.robot.subsystems.objectivetracker;

import static frc.robot.util.sim.HarvestHavocPantryLevel.CARROT_SIZE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

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

  // not gonna track number of pieces because that's annoying
  @Getter private GamePiece traderHeld;
  @Getter private GamePiece gripperHeld;

  // x positions of each game piece in pantry
  @Getter private final List<List<Double>> pantry = new ArrayList<>(3);

  // pieces vision has seen in the pantry but not confirmed yet
  private final List<PantryCandidate> pantryCandidates = new ArrayList<>();

  private final int[] pantryCakes = new int[3];

  @Getter private int ovenCarrots;
  @Getter private int cakesCollected;

  @Getter private int ovenCakes;

  private int ovenDumpPieces;
  private boolean ovenDumpIsCake;
  private final Debouncer traderEmptyDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  private int heldScoringIndex;

  private final int[] freeSlots = new int[3];
  private final int[] pantryCounts = new int[3];
  private final int[] loggedPantryCakes = new int[3];

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
                  if (collectingCake) {
                    gripperHeld = GamePiece.CARROT_CAKE;
                    cakesCollected++;
                  } else {
                    gripperHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  int scoringIndex = heldScoringIndex;
                  if (scoringIndex > 0 && AutoAlign.getLastTarget() == AutoAlign.Target.PANTRY) {
                    addPantryPiece(scoringIndex - 1, drive.getPose().getX());
                    if (gripperHeld == GamePiece.CARROT_CAKE) pantryCakes[scoringIndex - 1]++;
                  }
                  gripperHeld = null;
                  heldScoringIndex = 0;
                }));

    new Trigger(trader::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (collectingCake) {
                    traderHeld = GamePiece.CARROT_CAKE;
                    cakesCollected++;
                  } else {
                    traderHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(Commands.runOnce(() -> traderHeld = null));
  }

  @Override
  public void periodic() {
    // updatePantryFromVision();

    if (RobotUtil.isRedAlliance() != freeSlotsForRed) updateFreeSlots();

    if (gripper.isLoaded()) {
      heldScoringIndex = elevator.getSetpoint().scoringIndex;
    }

    int traderCount = trader.getNumGamePieces();
    boolean traderEmpty = traderEmptyDebounce.calculate(traderCount == 0);
    if (isNearOven()) {
      if (traderCount > 0) {
        ovenDumpPieces = Math.max(ovenDumpPieces, traderCount);
        ovenDumpIsCake = traderHeld == GamePiece.CARROT_CAKE;
      }
      if (traderEmpty && ovenDumpPieces > 0) {
        creditOvenDump(ovenDumpPieces);
      }
    } else if (ovenDumpPieces > 0) {
      creditOvenDump(Math.max(0, ovenDumpPieces - traderCount));
    }

    Logger.recordOutput("ObjectiveTracker/OvenCarrots", ovenCarrots);
    Logger.recordOutput("ObjectiveTracker/OvenCakes", ovenCakes);
    Logger.recordOutput("ObjectiveTracker/CakesCollected", cakesCollected);
    Logger.recordOutput("ObjectiveTracker/CakeReady", isCakeReady());
    for (int level = 0; level < 3; level++) {
      pantryCounts[level] = getPantryCount(level);
      loggedPantryCakes[level] = pantryCakes[level];
    }
    Logger.recordOutput("ObjectiveTracker/PantryCounts", pantryCounts);
    Logger.recordOutput("ObjectiveTracker/PantryCakes", loggedPantryCakes);
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
    ovenDumpPieces = 0;
    ovenDumpIsCake = false;
    heldScoringIndex = 0;
    collectingCake = false;
    updateFreeSlots();
    traderHeld = trader.isLoaded() ? GamePiece.CARROT : null;
    gripperHeld = gripper.isLoaded() ? GamePiece.CARROT : null;
  }

  private boolean isNearOven() {
    return drive
            .getPose()
            .getTranslation()
            .getSquaredDistance(
                AutoAlign.Target.OVEN.getPose(RobotUtil.isRedAlliance()).getTranslation())
        < NEAR_THRESHOLD;
  }

  public boolean isTrackingRP() {
    return DriverStation.getMatchType() == DriverStation.MatchType.Qualification;
  }

  private void creditOvenDump(int pieces) {
    if (ovenDumpIsCake) {
      ovenCakes += pieces;
    } else {
      ovenCarrots += pieces;
    }
    ovenDumpPieces = 0;
    ovenDumpIsCake = false;
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
