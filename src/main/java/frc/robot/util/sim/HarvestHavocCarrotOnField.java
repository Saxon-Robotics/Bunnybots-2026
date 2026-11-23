package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class HarvestHavocCarrotOnField extends GamePieceOnFieldSimulation {
  public static GamePieceOnFieldSimulation.GamePieceInfo HARVEST_HAVOC_CARROT_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          "Carrot",
          new Rectangle(Units.inchesToMeters(18), Units.inchesToMeters(6)),
          Inches.of(6),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  public static GamePieceOnFieldSimulation.GamePieceInfo HARVEST_HAVOC_CAKE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          "CakedUpCarrot",
          new Rectangle(Units.inchesToMeters(18), Units.inchesToMeters(6)),
          Inches.of(6),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  private static final GamePieceOnFieldSimulation.GamePieceInfo STANDING_CARROT_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          HARVEST_HAVOC_CARROT_INFO.type(),
          new Circle(Units.inchesToMeters(3)),
          Inches.of(18),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  private static final GamePieceOnFieldSimulation.GamePieceInfo STANDING_CAKE_INFO =
      new GamePieceOnFieldSimulation.GamePieceInfo(
          HARVEST_HAVOC_CAKE_INFO.type(),
          new Circle(Units.inchesToMeters(3)),
          Inches.of(18),
          Kilograms.of(0.294),
          2.8,
          4,
          0.15);

  private static final double STANDING_HALF_HEIGHT = Units.inchesToMeters(9);
  private static final double DINNER_TABLE_HEIGHT = Units.inchesToMeters(12);

  public static GamePieceOnFieldSimulation.GamePieceInfo getInfo(boolean isCake) {
    return isCake ? HARVEST_HAVOC_CAKE_INFO : HARVEST_HAVOC_CARROT_INFO;
  }

  private final boolean customRender;
  private final double renderZ;
  private final double renderPitch;

  public HarvestHavocCarrotOnField(boolean isCake, Pose2d initialPose) {
    super(getInfo(isCake), initialPose);
    customRender = false;
    renderZ = 0;
    renderPitch = 0;
  }

  private HarvestHavocCarrotOnField(
      GamePieceOnFieldSimulation.GamePieceInfo info,
      double z,
      Pose2d initialPose,
      double renderPitch) {
    super(info, () -> z, initialPose, new Translation2d());
    customRender = true;
    renderZ = z;
    this.renderPitch = renderPitch;
  }

  public static HarvestHavocCarrotOnField lyingAlongY(Translation2d position) {
    return new HarvestHavocCarrotOnField(
        HARVEST_HAVOC_CARROT_INFO,
        HARVEST_HAVOC_CARROT_INFO.gamePieceHeight().in(Meters) / 2,
        new Pose2d(position, Rotation2d.kCCW_90deg),
        0);
  }

  public static HarvestHavocCarrotOnField standing(Translation2d position) {
    return new HarvestHavocCarrotOnField(
        STANDING_CARROT_INFO,
        STANDING_HALF_HEIGHT,
        new Pose2d(position, Rotation2d.kZero),
        Math.PI / 2);
  }

  public static HarvestHavocCarrotOnField cakeOnDinnerTable(Translation2d position) {
    HarvestHavocCarrotOnField cake =
        new HarvestHavocCarrotOnField(
            STANDING_CAKE_INFO,
            DINNER_TABLE_HEIGHT + STANDING_HALF_HEIGHT,
            new Pose2d(position, Rotation2d.kZero),
            Math.PI / 2);
    cake.getFixture(0).setSensor(true);
    return cake;
  }

  @Override
  public Pose3d getPose3d() {
    if (!customRender) return super.getPose3d();
    Pose2d pose = getPoseOnField();
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        renderZ,
        new Rotation3d(0, renderPitch, pose.getRotation().getRadians()));
  }
}
