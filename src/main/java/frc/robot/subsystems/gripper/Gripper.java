// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTracer;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
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

  public Command intake() {
    return startRun(
            () -> roller.runVelocity(-GripperConstants.RPS),
            () -> {
              if (loadedDebounce.calculate(
                  roller.getVelocityRPS() < GripperConstants.VELOCITY_THRESHOLD
                      && roller.getStatorCurrentAmps() > GripperConstants.STATOR_THRESHOLD)) {
                isLoaded = true;
              }
            })
        .finallyDo(roller::stop);
  }

  public Command eject() {
    return startEnd(
        () -> roller.runVoltage(GripperConstants.VOLTAGE),
        () -> {
          roller.stop();
          isLoaded = false;
        });
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }
}
