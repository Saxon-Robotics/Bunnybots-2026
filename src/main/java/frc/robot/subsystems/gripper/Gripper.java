// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTracer;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.lasercan.LaserCanIO;
import frc.robot.util.io.sensors.lasercan.LaserCanIOInputsAutoLogged;
import frc.robot.util.sim.SimulationManager;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final Roller roller;
  private final LaserCanIO beambreak;
  private final LaserCanIOInputsAutoLogged beambreakInputs = new LaserCanIOInputsAutoLogged();

  private final Debouncer beambreakDebounce = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

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

    beambreak =
        switch (Constants.currentMode) {
          case REAL -> ignored -> {};
          case SIM -> LaserCanIO.beambreakSim(
              () -> SimulationManager.getInstance().isGripperLoaded(),
              GripperConstants.BEAMBREAK_THRESHOLD);
          case REPLAY -> inputs -> {};
        };
  }

  @Override
  public void periodic() {
    roller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Gripper/DistanceSensor", beambreakInputs);
    isLoaded =
        beambreakDebounce.calculate(
            beambreakInputs.measurementValid
                && beambreakInputs.distanceMillimeters <= GripperConstants.BEAMBREAK_THRESHOLD);
    LoggedTracer.record("Gripper/Periodic");
  }

  public Command intake() {
    return startEnd(() -> roller.runVelocity(-GripperConstants.RPS), roller::stop);
  }

  public Command eject() {
    return startEnd(() -> roller.runVoltage(GripperConstants.VOLTAGE), roller::stop);
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }
}
