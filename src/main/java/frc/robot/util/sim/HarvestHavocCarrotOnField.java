package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class HarvestHavocCarrotOnField extends GamePieceOnFieldSimulation {
  public static GamePieceOnFieldSimulation.GamePieceInfo HARVEST_HAVOC_CARROT_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          "Carrot",
          new Rectangle(Units.inchesToMeters(6), Units.inchesToMeters(18)),
          Inches.of(6),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  public static GamePieceOnFieldSimulation.GamePieceInfo HARVEST_HAVOC_CAKE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          "CakedUpCarrot",
          new Rectangle(Units.inchesToMeters(6), Units.inchesToMeters(18)),
          Inches.of(6),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  public static GamePieceOnFieldSimulation.GamePieceInfo getInfo(boolean isCake) {
    return isCake ? HARVEST_HAVOC_CAKE_INFO : HARVEST_HAVOC_CARROT_INFO;
  }

  public HarvestHavocCarrotOnField(boolean isCake, Pose2d initialPose) {
    super(getInfo(isCake), initialPose);
  }
}
