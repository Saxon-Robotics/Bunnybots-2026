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
import java.io.IOException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SelfDriving {
  private LoggedDashboardChooser<String> chooser;

  private DecisionTree.Node tree;

  private Command autoAlign;

  public SelfDriving(
      ObjectiveTracker tracker, Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    autoAlign = DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget).asProxy();

    Command scoreL1 =
        driveAndAlign(drive, tracker.getPantryPose(0, drive.getPose()))
            .andThen(elevator.l1())
            .andThen(gripper.eject().until(() -> !gripper.isLoaded()))
            .andThen(elevator.stow());
    Command scoreL2 =
        driveAndAlign(drive, tracker.getPantryPose(1, drive.getPose()))
            .andThen(elevator.l2())
            .andThen(gripper.eject().until(() -> !gripper.isLoaded()))
            .andThen(elevator.stow());
    Command scoreL3 =
        driveAndAlign(drive, tracker.getPantryPose(2, drive.getPose()))
            .andThen(elevator.l3())
            .andThen(gripper.eject().until(() -> !gripper.isLoaded()))
            .andThen(elevator.stow());
    Command scoreOven =
        driveAndAlign(drive, OVEN).andThen(trader.eject().until(() -> !trader.isLoaded()));
    Command collectRamp =
        driveAndAlign(drive, RAMP)
            .andThen(gripper.intake().until(gripper::isLoaded))
            .deadlineFor(elevator.ramp())
            .andThen(elevator.stow());
    Command traderCollectDepot =
        driveAndAlign(drive, DEPOT)
            .andThen(Commands.waitUntil(trader::isFullyLoaded)); // add intake until
    Command elevatorCollectDepot =
        driveAndAlign(drive, DEPOT)
            .andThen(gripper.intake().until(gripper::isLoaded))
            .deadlineFor(elevator.ramp())
            .andThen(elevator.stow());
    Command park = driveAndAlign(drive, TABLE);

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

    changeStrategy(chooser.get());
  }

  private Command driveAndAlign(Drive drive, Target target) {
    return drive.pathfindToPose(AutoAlign.setTarget(target, drive.getPose())).andThen(autoAlign);
  }

  private Command driveAndAlign(Drive drive, Pose2d target) {
    return drive
        .pathfindToPose(target)
        .andThen(() -> AutoAlign.setTargetPose(target, drive.getPose()))
        .andThen(autoAlign);
  }

  public Command routine() {
    return Commands.deferredProxy(tree::evaluate).repeatedly();
  }

  public void setUpChooser(LoggedDashboardChooser<String> chooser) {
    chooser.addDefaultOption("None", null);
    for (String name : DecisionTree.getAllStrategyNames()) {
      chooser.addOption(name.substring(0, name.lastIndexOf(".")), name);
    }
    chooser.onChange(this::changeStrategy);
    this.chooser = chooser;
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
