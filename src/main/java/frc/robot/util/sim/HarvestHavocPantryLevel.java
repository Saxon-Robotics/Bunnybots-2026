package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.Goal;

public class HarvestHavocPantryLevel extends Goal {
  private static final Angle ANGLE_TOLERANCE = Degrees.of(10);

  public static final Translation2d bluePantry =
      new Translation2d(-7.928 - 1.194, 4.03).plus(Constants.FieldConstants.ORIGIN);
  public static final Translation2d redPantry =
      new Translation2d(5.54 + 1.194, 4.03).plus(Constants.FieldConstants.ORIGIN);
  public static final Translation3d[] heights =
      new Translation3d[] {
        new Translation3d(0, 0, 0.830), // L1
        new Translation3d(0, 0, 1.186), // L2
        new Translation3d(0, 0, 1.541) // L3
      };

  /**
   *
   *
   * <h2>Returns the required pose of a pantry spot at the designated position.</h2>
   *
   * @param isBlue Whether the position is on the blue side or the red side.
   * @param level The level of the pantry (0 indexed). Range of 0-2.
   * @return The pose of a pantry spot with the specified stats.
   */
  public static Translation3d getPoseOfPantryAt(boolean isBlue, int level) {
    return new Translation3d(isBlue ? bluePantry : redPantry).plus(heights[level]);
  }

  private static final int MIN_LEVEL = 0;
  private static final int MAX_LEVEL = 2;
  public static final double CARROT_SIZE = 0.4572;

  public final int level;

  private final List<Pose3d> occupied;
  private final List<Pose3d> pieces = new ArrayList<>(5);

  private final double minZMeters;
  private final double maxZMeters;

  /**
   *
   *
   * <h2>Creates a singular reef branch at the specified location </h2>
   *
   * @param arena The host arena of this pantry.
   * @param isBlue Whether the position is on the blue pantry or the red pantry.
   * @param level The level of the pantry (0 indexed). Range of 0-2.
   */
  public HarvestHavocPantryLevel(
      Arena2026Bunnybots arena,
      boolean isBlue,
      int level,
      boolean forCakes,
      List<Pose3d> occupied) {
    super(
        arena,
        Meters.of(0.4776 * 5),
        Meters.of(0.160),
        Meters.of(0.222),
        HarvestHavocCarrotOnField.getInfo(forCakes).type(),
        getPoseOfPantryAt(isBlue, level),
        isBlue,
        5,
        false);

    if (level < MIN_LEVEL || level > MAX_LEVEL) {
      throw new IllegalArgumentException(
          "Invalid pantry level: "
              + level
              + " (must be between "
              + MIN_LEVEL
              + " and "
              + MAX_LEVEL
              + ")");
    }

    this.level = level;
    this.occupied = occupied;

    minZMeters = position.getZ();
    maxZMeters = position.getZ() + height.in(Meters);

    // Set initial rotation checker
    // todo check 90 degree with game piece
    setNeededAngle(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero()), ANGLE_TOLERANCE);
    withCustomPositionChecker(
        position -> {
          /* basic within bounding box,
          find distance between x value of carrot just scored and every already scored carrot, if the distance is between the two is less then the carrot size return false
          add variable to the list, with set y and z.
          return true
           */
          if (!xyBox.contains(new Vector2(position.getX(), position.getY()))
              || position.getZ() < minZMeters
              || position.getZ() > maxZMeters) {
            return false;
          }
          for (Pose3d carrot : occupied) {
            if (Math.abs(carrot.getX() - position.getX()) < CARROT_SIZE) {
              return false;
            }
          }
          Pose3d carrotInPantry =
              new Pose3d(
                  position.getX() + CARROT_SIZE / 4,
                  this.position.getY(),
                  this.position.getZ(),
                  new Rotation3d(Rotation2d.kZero));
          occupied.add(carrotInPantry);
          pieces.add(carrotInPantry);

          return true;
        });
  }

  /**
   *
   *
   * <h2>Gives the pose of the pantry slot.</h2>
   *
   * @return This position of this slot as a Pose3d.
   */
  public Pose3d getPose() {
    return new Pose3d(position, new Rotation3d(Rotation2d.kZero));
  }

  private static final int[] POINTS_BY_LEVEL = {3, 4, 5};

  @Override
  protected void addPoints() {
    System.out.println(
        "Carrot scored on level: "
            + (level + 1)
            + " on the "
            + (isBlue ? "Blue" : "Red")
            + " pantry");
    arena.addValueToMatchBreakdown(
        isBlue, "Auto/CarrotScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);
    arena.addValueToMatchBreakdown(isBlue, "CarrotScoredOnLevel " + (level + 1), 1);

    arena.addToScore(isBlue, POINTS_BY_LEVEL[level]);
  }

  @Override
  public void draw(List<Pose3d> drawList) {
    drawList.addAll(pieces);
  }

  public void draw(String type, List<Pose3d> drawList) {
    if (type.equals(gamePieceType)) draw(drawList);
  }

  @Override
  public void clear() {
    super.clear();
    occupied.clear();
    pieces.clear();
  }
}
