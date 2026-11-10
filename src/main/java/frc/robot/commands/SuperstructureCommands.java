package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;

public class SuperstructureCommands {
  private static final double WAIT_TIMEOUT_SEC = 1.5;

  public static Command intake(Elevator elevator, Arm arm, Gripper gripper) {
    return Commands.either(
            arm.ground(),
            arm.ramp(),
            () -> elevator.getSetpoint() == ElevatorConstants.Setpoint.STOWED)
        .alongWith(gripper.intake());
  }

  public static Command outtake(Elevator elevator, Arm arm, Gripper gripper) {
    return Commands.either(arm.pantry(), arm.oven(), () -> elevator.getSetpoint().scoringIndex > 0)
        .alongWith(
            Commands.waitUntil(arm::hasReachedSetpoint)
                .withTimeout(WAIT_TIMEOUT_SEC)
                .andThen(gripper.eject()));
  }
}
