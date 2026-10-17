package frc.robot.subsystems.objectivetracker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SelfDriving;
import java.util.function.BooleanSupplier;

public class Planner {
  private static Planner instance;

  public static Planner getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Planner has not been initialized!");
    }
    return instance;
  }

  public static Planner initialize(ObjectiveTracker tracker, SelfDriving commands) {
    instance = new Planner(tracker, commands);
    return instance;
  }

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

  private Planner(ObjectiveTracker tracker, SelfDriving commands) {
    // using conditions from the tracker and commands from SelfDriving, we build a tree to define
    // all logic
    this.tree = null;
  }

  public Command getNextCommand() {
    return tree.evaluate();
  }
}
