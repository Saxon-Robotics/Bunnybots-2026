package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;

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
  public final HarvestHavocOvenSimulation blueOvenCarrot;
  public final HarvestHavocOvenSimulation blueOvenCake;
  public final HarvestHavocOvenSimulation redOvenCarrot;
  public final HarvestHavocOvenSimulation redOvenCake;

  public Arena2026Bunnybots() {
    super(new HarvestHavocFieldObstacleMap());

    bluePantry = new HarvestHavocPantrySimulation(this, true);
    super.addCustomSimulation(bluePantry);

    redPantry = new HarvestHavocPantrySimulation(this, false);
    super.addCustomSimulation(redPantry);

    blueOvenCarrot = new HarvestHavocOvenSimulation(this, true, false);
    blueOvenCake = new HarvestHavocOvenSimulation(this, true, true);
    super.addCustomSimulation(blueOvenCarrot);
    super.addCustomSimulation(blueOvenCake);

    redOvenCarrot = new HarvestHavocOvenSimulation(this, false, false);
    redOvenCake = new HarvestHavocOvenSimulation(this, false, true);
    super.addCustomSimulation(redOvenCarrot);
    super.addCustomSimulation(redOvenCake);
  }

  private static final double CARROT_COLUMN_X = Units.feetToMeters(7);
  private static final double CARROT_SPACING_Y = Units.feetToMeters(3.5);
  private static final double TABLE_CAKE_X = Units.inchesToMeters(25.7);

  @Override
  public void placeGamePiecesOnField() {
    for (double side : new double[] {-1, 1}) {
      double x = side * CARROT_COLUMN_X;
      for (int i = -2; i <= 2; i++) {
        Translation2d position = new Translation2d(x, i * CARROT_SPACING_Y).plus(ORIGIN);
        super.addGamePiece(
            Math.abs(i) == 2
                ? HarvestHavocCarrotOnField.standing(position)
                : HarvestHavocCarrotOnField.lyingAlongY(position));
      }
      super.addGamePiece(
          HarvestHavocCarrotOnField.cakeOnDinnerTable(
              new Translation2d(side * TABLE_CAKE_X, 0).plus(ORIGIN)));
    }

    setupValueForMatchBreakdown("CarrotScoredInOven");
    setupValueForMatchBreakdown("Auto/CarrotScoredInAuto");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 1");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 2");
    setupValueForMatchBreakdown("CarrotScoredOnLevel 3");
  }

  @Override
  public synchronized List<GamePiece> getGamePiecesByType(String type) {
    return gamePieces.stream().filter(gamePiece -> type.equals(gamePiece.getType())).toList();
  }

  @Override
  public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
    List<Pose3d> poses = super.getGamePiecesPosesByType(type);

    bluePantry.draw(type, poses);
    redPantry.draw(type, poses);

    return poses;
  }

  @Override
  public synchronized void clearGamePieces() {
    super.clearGamePieces();
    bluePantry.clearPantry();
    redPantry.clearPantry();
  }
}
