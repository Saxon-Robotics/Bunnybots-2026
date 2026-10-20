package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.subsystems.trader.Trader;
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

  public SelfDriving(
      ObjectiveTracker tracker, Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    // commands needed: scoreL1/2/3, scoreOven, collectRamp, collectDepot, park
    // commands will call pathfinder then use autopilot for final align
    // stopWithX might be tech?
    tree = null;
  }

  public Command routine() {
    return Commands.deferredProxy(tree::evaluate).repeatedly();
  }
}
