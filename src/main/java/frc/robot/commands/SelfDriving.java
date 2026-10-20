package frc.robot.commands;

import static frc.robot.util.AutoAlign.Target.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.subsystems.trader.Trader;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlign.Target;
import java.util.function.BooleanSupplier;

public class SelfDriving {
  private static class Node {
    private final BooleanSupplier condition;
    private final Node yesBranch;
    private final Node noBranch;
    private final Command leafCommand;

    // decision node
    public Node(BooleanSupplier condition, Node yesBranch, Node noBranch) {
      this.condition = condition;
      this.yesBranch = yesBranch;
      this.noBranch = noBranch;
      this.leafCommand = null;
    }

    // leaf node
    public Node(Command leafCommand) {
      this.condition = null;
      this.yesBranch = null;
      this.noBranch = null;
      this.leafCommand = leafCommand;
    }

    public boolean isLeaf() {
      return leafCommand != null;
    }

    public Command evaluate() {
      if (this.isLeaf()) {
        return this.leafCommand;
      }

      if (condition.getAsBoolean()) {
        return yesBranch.evaluate();
      } else {
        return noBranch.evaluate();
      }
    }
  }

  private final Node tree;

  private Command autoAlign;

  public SelfDriving(
      ObjectiveTracker tracker, Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    // commands needed: scoreL1/2/3, scoreOven, collectRamp, collectDepot, park
    // commands will call pathfinder then use autopilot for final align
    // stopWithX might be tech?
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
    
    tree = null;
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
}
