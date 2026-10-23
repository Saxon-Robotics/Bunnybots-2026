package frc.robot.commands;

import static frc.robot.util.AutoAlign.Target.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.trader.Trader;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlign.Target;
import frc.robot.util.DecisionTree;
import frc.robot.util.LoggedTracer;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SelfDriving {
  private DecisionTree.Node tree = DecisionTree.EMPTY;

  public SelfDriving(
      ObjectiveTracker tracker, Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    Command scoreL1 = scorePantry(0, tracker, drive, elevator, gripper).withName("Score L1");
    Command scoreL2 = scorePantry(1, tracker, drive, elevator, gripper).withName("Score L2");
    Command scoreL3 = scorePantry(2, tracker, drive, elevator, gripper).withName("Score L3");
    Command scoreOven =
        driveAndAlign(drive, OVEN)
            .andThen(trader.eject().until(() -> !trader.isLoaded()))
            .withName("Score Oven");
    Command collectRamp =
        driveAndAlign(drive, RAMP)
            .andThen(gripper.intake().until(gripper::isLoaded))
            .deadlineFor(elevator.ramp())
            .andThen(elevator.stow())
            .withName("Collect Ramp");
    Command traderCollectDepot =
        driveAndAlign(drive, DEPOT)
            .andThen(Commands.waitUntil(trader::isFullyLoaded))
            .withName("Collect Depot - Trader"); // add intake until
    Command elevatorCollectDepot =
        driveAndAlign(drive, DEPOT)
            .andThen(gripper.intake().until(gripper::isLoaded))
            .deadlineFor(elevator.ramp())
            .andThen(elevator.stow())
            .withName("Collect Depot - Elevator");
    Command park = driveAndAlign(drive, TABLE).withName("Park");

    DecisionTree.registerCommand("scoreL1", scoreL1);
    DecisionTree.registerCommand("scoreL2", scoreL2);
    DecisionTree.registerCommand("scoreL3", scoreL3);
    DecisionTree.registerCommand("scoreOven", scoreOven);
    DecisionTree.registerCommand("collectRamp", collectRamp);
    DecisionTree.registerCommand("traderCollectDepot", traderCollectDepot);
    DecisionTree.registerCommand("elevatorCollectDepot", elevatorCollectDepot);
    DecisionTree.registerCommand("park", park);

    DecisionTree.registerCondition("gripperLoaded", gripper::isLoaded);
    DecisionTree.registerCondition(
        "gripperHasCake", () -> tracker.getGripperHeld() == GamePiece.CARROT_CAKE);
    DecisionTree.registerCondition("traderLoaded", trader::isLoaded);
    DecisionTree.registerCondition("traderFull", trader::isFullyLoaded);
    DecisionTree.registerCondition("cakeReady", tracker::isCakeReady);
    DecisionTree.registerCondition("stockedUp", tracker::isStockedUp);
    DecisionTree.registerCondition("pantryL1Open", () -> tracker.hasPantrySlot(0));
    DecisionTree.registerCondition("pantryL2Open", () -> tracker.hasPantrySlot(1));
    DecisionTree.registerCondition("pantryL3Open", () -> tracker.hasPantrySlot(2));
  }

  private Command driveAndAlign(Drive drive, Target target) {
    return drive
        .defer(() -> drive.pathfindToPose(AutoAlign.setTarget(target, drive.getPose())))
        .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));
  }

  private Command driveAndAlign(Drive drive, Supplier<Pose2d> target) {
    return drive
        .defer(
            () ->
                drive
                    .pathfindToPose(target.get())
                    .andThen(() -> AutoAlign.setTargetPose(target.get(), drive.getPose())))
        .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));
  }

  private Command scorePantry(
      int level, ObjectiveTracker tracker, Drive drive, Elevator elevator, Gripper gripper) {
    return driveAndAlign(drive, () -> tracker.getPantryPose(level, drive.getPose()))
        .andThen(elevator.l1())
        .andThen(gripper.eject().until(() -> !gripper.isLoaded()))
        .andThen(elevator.stow());
  }

  public Command routine() {
    return Commands.deferredProxy(
            () -> {
              List<String> traversalPath = new ArrayList<>();
              Command next = tree.evaluate(traversalPath);
              Logger.recordOutput(
                  "SelfDriving/TraversalPath", traversalPath.toArray(new String[0]));
              LoggedTracer.record("SelfDriving/Decision");
              return next;
            })
        .repeatedly();
  }

  public void setUpChooser(LoggedDashboardChooser<String> chooser) {
    chooser.addDefaultOption("None", null);
    for (String name : DecisionTree.getAllStrategyNames()) {
      chooser.addOption(name.substring(0, name.lastIndexOf(".")), name);
    }
    chooser.onChange(this::changeStrategy);
    changeStrategy(chooser.get());
  }

  public void changeStrategy(String name) {
    if (name == null) {
      tree = DecisionTree.EMPTY;
      return;
    }
    try {
      tree = DecisionTree.fromJson(name);
    } catch (IOException e) {
      System.err.println("Strategy not found: " + name);
      e.printStackTrace();
      tree = DecisionTree.EMPTY;
    }
  }
}
