package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

public class OldHarvestHavocPantrySimulation implements SimulatedArena.Simulatable {
  protected final List<OldHarvestHavocPantrySpot> spots;
  Pose3d[] pantrySpotPoses;

  /**
   *
   *
   * <h2>Creates a pantry of the specified color.</h2>
   *
   * @param arena The host arena of this pantry.
   * @param isBlue Whether this is the blue pantry or the red one.
   */
  OldHarvestHavocPantrySimulation(Arena2026Bunnybots arena, boolean isBlue) {
    spots = new ArrayList<>(15);
    pantrySpotPoses = new Pose3d[15];
    for (int level = 0; level < 3; level++) {
      for (int col = 0; col < 5; col++) {
        OldHarvestHavocPantrySpot spot = new OldHarvestHavocPantrySpot(arena, isBlue, level, col);
        spots.add(spot);

        pantrySpotPoses[(level * 5 + col)] = spot.getPose();
      }
    }

    StructArrayPublisher<Pose3d> pantryPub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic(
                "/SmartDashboard/MapleSim/Goals/" + (isBlue ? "BluePantry" : "RedPantry"),
                Pose3d.struct)
            .publish();

    pantryPub.set(pantrySpotPoses);
  }

  public void draw(List<Pose3d> coralPosesToDisplay) {
    for (OldHarvestHavocPantrySpot spot : spots) {
      spot.draw(coralPosesToDisplay);
    }
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    for (OldHarvestHavocPantrySpot spot : spots) {
      spot.simulationSubTick(subTickNum);
    }
  }

  /**
   *
   *
   * <h2>Resets the pantry to its original state.</h2>
   */
  public void clearPantry() {
    for (OldHarvestHavocPantrySpot spot : spots) {
      spot.clear();
    }
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
  public int[][] getSpots() {
    int[][] toReturn = new int[3][5];
    for (OldHarvestHavocPantrySpot spot : spots) {
      toReturn[spot.level][spot.column] = spot.getGamePieceCount();
    }
    return toReturn;
  }
}
