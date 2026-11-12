package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

public class SuperstructureCommands {
  private static final double WAIT_TIMEOUT_SEC = 1.5;

  public static void setupTriggers(Arm arm, Gripper gripper) {
    // holding while loaded
    new Trigger(gripper::isLoaded).whileTrue(gripper.holdStatic());
    // holding harder while pivot is moving
    new Trigger(() -> gripper.isLoaded() && !arm.hasReachedSetpoint())
        .whileTrue(gripper.holdMoving());
  }

  public static Command intake(Arm arm, Gripper gripper) {
    return arm.ramp().alongWith(gripper.intake());
  }

  public static Command intakeFromGround(Elevator elevator, Arm arm, Gripper gripper) {
    return elevator.stow().andThen(arm.ground().alongWith(gripper.intake()));
  }

  public static Command outtake(Elevator elevator, Arm arm, Gripper gripper) {
    return Commands.either(arm.pantry(), arm.oven(), () -> elevator.getSetpoint().scoringIndex > 0)
        .alongWith(
            Commands.waitUntil(arm::hasReachedSetpoint)
                .withTimeout(WAIT_TIMEOUT_SEC)
                .andThen(gripper.eject()));
  }
}
