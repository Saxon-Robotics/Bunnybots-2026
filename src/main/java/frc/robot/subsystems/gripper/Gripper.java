// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTracer;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.sim.SimulationManager;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Gripper extends SubsystemBase {
  private final Roller roller;

  private final Alert topDisconnectedAlert =
      new Alert("Top gripper motor disconnected.", Alert.AlertType.kError);
  private final Alert bottomDisconnectedAlert =
      new Alert("Bottom gripper motor disconnected.", Alert.AlertType.kError);
  private final Debouncer loadedDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  @Getter
  @AutoLogOutput(key = "Gripper/HasGamePiece")
  private boolean isLoaded;

  private boolean idle;
  private boolean movingHold;
  private boolean spunUp;

  public Gripper() {
    RollerIO io =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.GRIPPER_TOP,
                  GripperConstants.MOTOR_CONFIG)
              .addFollower(Constants.CANConstants.GRIPPER_BOTTOM, MotorAlignmentValue.Aligned)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX44(1),
              new MotorIO.RotationalMechanismConstraints(1, GripperConstants.MOI, 0, 0, 0, 0),
              0.5,
              0,
              1);
          case REPLAY -> new RollerIO() {};
        };
    roller = new Roller("Gripper", io, () -> true);
  }

  @Override
  public void periodic() {
    roller.periodic();
    topDisconnectedAlert.set(!roller.isConnected());
    bottomDisconnectedAlert.set(!roller.getFollowerConnected()[0]);

    LoggedTracer.record("Gripper/Periodic");
  }

  private void stop() {
    roller.stop();
    idle = true;
  }

  public Command intake() {
    return startRun(
            () -> {
              idle = false;
              spunUp = false;
              roller.runVelocity(-GripperConstants.RPS);
            },
            () -> {
              if (loadedDebounce.calculate(hasGamePiece())) {
                isLoaded = true;
              }
            })
        .finallyDo(this::stop);
  }

  public Command holdStatic() {
    return Commands.waitUntil(() -> idle && !movingHold)
        .andThen(
            startEnd(
                () -> roller.runVoltage(GripperConstants.STATIC_HOLDING_VOLTAGE), roller::stop));
  }

  public Command holdMoving() {
    return Commands.waitUntil(() -> idle)
        .andThen(
            startEnd(
                () -> {
                  movingHold = true;
                  roller.runVoltage(GripperConstants.MOVING_HOLDING_VOLTAGE);
                },
                roller::stop));
  }

  public Command eject() {
    return startEnd(
        () -> {
          idle = false;
          roller.runVoltage(GripperConstants.VOLTAGE);
        },
        () -> {
          stop();
          isLoaded = false;
        });
  }

  private boolean hasGamePiece() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return SimulationManager.getInstance().isGripperLoaded();
    }
    double speed = Math.abs(roller.getVelocityRPS());
    if (speed > GripperConstants.SPUN_UP_RPS) spunUp = true;
    return spunUp
        && speed < GripperConstants.VELOCITY_THRESHOLD
        && Math.abs(roller.getStatorCurrentAmps()) > GripperConstants.STATOR_THRESHOLD;
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }
}
