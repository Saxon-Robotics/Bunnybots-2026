package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.util.AutoAlign;

public class Autos {
  public static Command scorePantry(
      int level, Drive drive, Elevator elevator, Arm arm, Gripper gripper) {
    return Commands.runOnce(
            () ->
                AutoAlign.setTargetPose(
                    AutoAlign.Target.PANTRY,
                    ObjectiveTracker.getInstance().getPantryPose(level, drive.getPose())))
        .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget))
        .andThen(
            switch (level) {
              case 0 -> elevator.l1();
              case 1 -> elevator.l2();
              case 2 -> elevator.l3();
              default -> Commands.none();
            })
        .andThen(
            SuperstructureCommands.outtake(elevator, arm, gripper)
                .withTimeout(GripperConstants.OUTTAKE_TIMEOUT));
  }
}
