package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.List;
import org.ironmaple.simulation.Goal;

public class HarvestHavocPantrySpot extends Goal {
  private static final Angle ANGLE_TOLERANCE = Degrees.of(10);

  public static final Translation2d bluePantry =
      new Translation2d(-7.928, 4.03).plus(Constants.FieldConstants.ORIGIN);
  public static final Translation2d redPantry =
      new Translation2d(5.54, 4.03).plus(Constants.FieldConstants.ORIGIN);
  public static final Translation3d[] heights =
      new Translation3d[] {
        new Translation3d(0, 0, 0.830), // L1
        new Translation3d(0, 0, 1.186), // L2
        new Translation3d(0, 0, 1.541) // L3
      };
  public static final Translation3d columnIncrement = new Translation3d(0.4776, 0, 0);

  /**
   *
   *
   * <h2>Returns the required pose of a pantry spot at the designated position.</h2>
   *
   * @param isBlue Whether the position is on the blue side or the red side.
   * @param level The level of the pantry (0 indexed). Range of 0-2.
   * @param col The column of the pantry slot (0 indexed). Range of 0-4.
   * @return The pose of a pantry spot with the specified stats.
   */
  public static Translation3d getPoseOfPantryAt(boolean isBlue, int level, int col) {
    return new Translation3d(isBlue ? bluePantry : redPantry)
        .plus(heights[level])
        .plus(columnIncrement.times(col));
  }

  private static final int MIN_LEVEL = 0;
  private static final int MAX_LEVEL = 2;

  public final int level;
  public final int column;

  /**
   *
   *
   * <h2>Creates a singular reef branch at the specified location </h2>
   *
   * @param arena The host arena of this pantry.
   * @param isBlue Whether the position is on the blue pantry or the red pantry.
   * @param level The level of the pantry (0 indexed). Range of 0-2.
   * @param column The column of the pantry slot (0 indexed). Range of 0-4.
   */
  public HarvestHavocPantrySpot(Arena2026Bunnybots arena, boolean isBlue, int level, int column) {
    super(
        arena,
        columnIncrement.getMeasureX(),
        Meters.of(0.160),
        Meters.of(0.222),
        "Carrot",
        getPoseOfPantryAt(isBlue, level, column),
        isBlue,
        1,
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
    this.column = column;

    // Set initial rotation checker
    // todo check 90 degree with game piece
    setNeededAngle(new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero()), ANGLE_TOLERANCE);
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
    if (this.gamePieceCount > 0) {
      drawList.add(getPose());
    }
  }
}
