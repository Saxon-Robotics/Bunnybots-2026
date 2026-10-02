package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.*;
import org.ironmaple.simulation.Goal;

public class HarvestHavocOvenSimulation extends Goal {
  protected static final Translation3d blueOvenPose =
      new Translation3d(Constants.FieldConstants.BLUE_OVEN.getTranslation());
  protected static final Translation3d redOvenPose =
      new Translation3d(Constants.FieldConstants.RED_OVEN.getTranslation());

  public HarvestHavocOvenSimulation(Arena2026Bunnybots arena, boolean isBlue) {
    super(
        arena,
        Meters.of(0.762),
        Meters.of(0.55),
        Meters.of(0.381),
        "Carrot",
        isBlue ? blueOvenPose : redOvenPose,
        isBlue,
        true);

    StructPublisher<Pose3d> ovenPub =
        NetworkTableInstance.getDefault()
            .getStructTopic(
                "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BlueOven" : "RedOven"),
                Pose3d.struct)
            .publish();
    ovenPub.set(new Pose3d(position, new Rotation3d(0, 0, Math.PI)));
  }

  @Override
  protected void addPoints() {
    arena.addValueToMatchBreakdown(
        isBlue, "Auto/CarrotScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);
    arena.addValueToMatchBreakdown(isBlue, "CarrotScoredInOven", 1);

    arena.addToScore(isBlue, 2);
  }

  /**
   * Drawing on field isn't supported with oven because game pieces go off the field to HP
   *
   * @param drawList a list of {@link Pose3d} objects used to visualize the positions of the game
   *     pieces on AdvantageScope
   */
  @Override
  public void draw(List<Pose3d> drawList) {}
}
