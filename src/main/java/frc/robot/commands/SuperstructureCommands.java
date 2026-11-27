package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

public class SuperstructureCommands {
  private static final double WAIT_TIMEOUT_SEC = 1.0;

  public static Command intake(Arm arm, Gripper gripper) {
    return gripper.intake();
  }

  public static Command intakeFromGround(Elevator elevator, Arm arm, Gripper gripper) {
    return elevator.stow().andThen(arm.ground().alongWith(gripper.intake()));
  }

  public static Command outtake(Arm arm, Gripper gripper) {
    return arm.outtake()
        .alongWith(
            Commands.waitUntil(arm::hasReachedSetpoint)
                .withTimeout(WAIT_TIMEOUT_SEC)
                .andThen(gripper.eject()));
  }
}
