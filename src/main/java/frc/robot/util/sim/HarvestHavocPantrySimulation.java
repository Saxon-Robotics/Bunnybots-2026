package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

public class HarvestHavocPantrySimulation implements SimulatedArena.Simulatable {
  protected final List<HarvestHavocPantryLevel> levels;
  Pose3d[] pantryLevelPoses;

  /**
   *
   *
   * <h2>Creates a pantry of the specified color.</h2>
   *
   * @param arena The host arena of this pantry.
   * @param isBlue Whether this is the blue pantry or the red one.
   */
  HarvestHavocPantrySimulation(Arena2026Bunnybots arena, boolean isBlue) {
    levels = new ArrayList<>(15);
    pantryLevelPoses = new Pose3d[3];
    for (int level = 0; level < 3; level++) {
      HarvestHavocPantryLevel levelGoal = new HarvestHavocPantryLevel(arena, isBlue, level, false);
      levels.add(levelGoal);
      levels.add(new HarvestHavocPantryLevel(arena, isBlue, level, true));

      pantryLevelPoses[level] = levelGoal.getPose();
    }

    StructArrayPublisher<Pose3d> pantryPub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic(
                "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BluePantry" : "RedPantry"),
                Pose3d.struct)
            .publish();

    pantryPub.set(pantryLevelPoses);
  }

  public void draw(List<Pose3d> carrotPosesToDisplay) {
    for (HarvestHavocPantryLevel level : levels) {
      level.draw(carrotPosesToDisplay);
    }
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    for (HarvestHavocPantryLevel level : levels) {
      level.simulationSubTick(subTickNum);
    }
  }

  /**
   *
   *
   * <h2>Resets the pantry to its original state.</h2>
   */
  public void clearPantry() {
    for (HarvestHavocPantryLevel level : levels) {
      level.clear();
    }
  }
}
