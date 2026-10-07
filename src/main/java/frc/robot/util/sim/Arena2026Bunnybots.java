package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h1>The playing field for the 2026 Blair Bunnybots Game: Harvest Havoc</h1>
 *
 * <p>This class represents the playing field for the 2026 Bunnybots game, Harvest Havoc.
 *
 * <p>It extends {@link SimulatedArena} and includes specific details of the Harvest Havoc game
 * environment.
 */
public class Arena2026Bunnybots extends SimulatedArena {
  public static final class HarvestHavocFieldObstacleMap extends FieldMap {
    // width is x, height is y, poses are in the center of obstacle
    public HarvestHavocFieldObstacleMap() {
      super();

      // blue wall
      super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FIELD_WIDTH.in(Meters)));

      // blue ramp
      super.addRectangularObstacle(
          2.343,
          0.632,
          new Pose2d(new Translation2d(-7.101, -1.739).plus(ORIGIN), Rotation2d.kZero));

      // blue pantry
      super.addRectangularObstacle(
          2.464,
          0.178,
          new Pose2d(new Translation2d(-6.734, 4.026).plus(ORIGIN), Rotation2d.kZero));

      // red wall
      super.addBorderLine(
          new Translation2d(FIELD_LENGTH.in(Meters), 0),
          new Translation2d(FIELD_LENGTH.in(Meters), FIELD_WIDTH.in(Meters)));

      // red ramp
      super.addRectangularObstacle(
          2.343,
          0.632,
          new Pose2d(new Translation2d(7.101, -1.739).plus(ORIGIN), Rotation2d.kZero));

      // red pantry
      super.addRectangularObstacle(
          2.464, 0.178, new Pose2d(new Translation2d(6.734, 4.026).plus(ORIGIN), Rotation2d.kZero));

      // upper walls
      super.addBorderLine(
          new Translation2d(FIELD_LENGTH.in(Meters), FIELD_WIDTH.in(Meters)),
          new Translation2d(0, FIELD_WIDTH.in(Meters)));

      // lower walls
      super.addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_LENGTH.in(Meters), 0));

      // dining table
      super.addRectangularObstacle(1.194, 1.194, new Pose2d(ORIGIN, Rotation2d.fromDegrees(45)));
    }
  }

  public final HarvestHavocPantrySimulation bluePantry;
  public final HarvestHavocPantrySimulation redPantry;
  public final HarvestHavocOvenSimulation blueOven;
  public final HarvestHavocOvenSimulation redOven;

  public Arena2026Bunnybots() {
    super(new HarvestHavocFieldObstacleMap());

    bluePantry = new HarvestHavocPantrySimulation(this, true);
    super.addCustomSimulation(bluePantry);

    redPantry = new HarvestHavocPantrySimulation(this, false);
    super.addCustomSimulation(redPantry);

    blueOven = new HarvestHavocOvenSimulation(this, true);
    super.addCustomSimulation(blueOven);

    redOven = new HarvestHavocOvenSimulation(this, false);
    super.addCustomSimulation(redOven);
  }

  @Override
  public void placeGamePiecesOnField() {
    //    Translation2d[] bluePositions = new Translation2d[] {
    //        new Translation2d(1.219, 5.855), new Translation2d(1.219, 4.026), new
    // Translation2d(1.219, 2.197),
    //    };
    //    for (Translation2d position : bluePositions) super.addGamePiece(new
    // ReefscapeCoralAlgaeStack(position));
    //
    //    Translation2d[] redPositions = Arrays.stream(bluePositions)
    //        .map(bluePosition ->
    //            new Translation2d(FieldMirroringUtils.FIELD_WIDTH - bluePosition.getX(),
    // bluePosition.getY()))
    //        .toArray(Translation2d[]::new);
    //    for (Translation2d position : redPositions) super.addGamePiece(new
    // ReefscapeCoralAlgaeStack(position));

    setupValueForMatchBreakdown("CarrotScoredInOven");
    setupValueForMatchBreakdown("Auto/CarrotScoredInAuto");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 1");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 2");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 3");
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    List<Pose3d> poses = super.getGamePiecesPosesByType(type);

    if (type.equals("Carrot")) {
      bluePantry.draw(poses);
      redPantry.draw(poses);
    }

    return poses;
  }

  @Override
  public synchronized void clearGamePieces() {
    super.clearGamePieces();
    bluePantry.clearPantry();
    redPantry.clearPantry();
  }

  /**
   * Obtains the amount of <strong>CARROT</strong> held in the <strong>PANTRY</strong>.
   *
   * <p>This method returns a 2D array of size 3 x 5, where each entry represents the number of
   * <strong>CARROT</strong> held in a particular spot.
   *
   * <p>The [i][j] entry in the array represents the number of <strong>CARROT</strong>(s) held in
   * the <code>j</code> slot (left to right) in the <code>i-1</code>th level.
   *
   * <p>For example, <code>getBranches()[2][3]</code> returns the number of CARROT held on slot 4 of
   * L2.
   *
   * <p>Note that each slot can only hold one <strong>CARROT</strong>.
   *
   * @return a 2D array where each entry represents the number of <strong>CARROT</strong> held on
   *     each branch
   */
  public int[][] getCarrots(DriverStation.Alliance side) {
    return side == DriverStation.Alliance.Red ? redPantry.getSpots() : bluePantry.getSpots();
  }
}
