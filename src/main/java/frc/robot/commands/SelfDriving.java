package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.objectivetracker.Planner;
import frc.robot.subsystems.trader.Trader;

public class SelfDriving {
  // individual commands representing one stage in a routine
  public final Command someCommand;

  public SelfDriving(Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    // all commands get made here
    someCommand = drive.idle(); // (placeholder)
  }

  public Command buildRoutine() {
    return Commands.deferredProxy(() -> Planner.getInstance().getNextCommand()).repeatedly();
  }
}
