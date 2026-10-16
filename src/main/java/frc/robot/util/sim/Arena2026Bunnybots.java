package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
