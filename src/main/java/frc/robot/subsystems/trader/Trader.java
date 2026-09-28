// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trader;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.lasercan.LaserCanIO;
import frc.robot.util.io.sensors.lasercan.LaserCanIOInputsAutoLogged;
import frc.robot.util.io.sensors.lasercan.LaserCanIOReal;
import frc.robot.util.sim.SimulationHelper;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Trader extends SubsystemBase {
  private final Roller roller;
  private final LaserCanIO beambreak;
  private final LaserCanIOInputsAutoLogged beambreakInputs = new LaserCanIOInputsAutoLogged();

  private final Debouncer beambreakDebounce = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

  @Getter
  @AutoLogOutput(key = "Trader/HasGamePiece")
  private boolean isLoaded;

  public Trader() {
    RollerIO leftIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.TRADER_LEFT,
                  TraderConstants.MOTOR_CONFIG)
              .addFollower(Constants.CANConstants.TRADER_RIGHT, MotorAlignmentValue.Opposed)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(1, TraderConstants.MOI, 0, 0, 0, 0),
              TraderConstants.KP,
              TraderConstants.KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    roller = new Roller("Trader", leftIO);

    beambreak =
        switch (Constants.currentMode) {
          case REAL -> new LaserCanIOReal(Constants.CANConstants.TRADER_LASERCAN);
          case SIM -> LaserCanIO.beambreakSim(
              () -> SimulationHelper.getInstance().isTraderLoaded(),
              TraderConstants.BEAMBREAK_THRESHOLD);
          case REPLAY -> inputs -> {};
        };
  }

  @Override
  public void periodic() {
    roller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Trader/DistanceSensor", beambreakInputs);
    isLoaded =
        beambreakDebounce.calculate(
            beambreakInputs.measurementValid
                && beambreakInputs.distanceMillimeters <= GripperConstants.BEAMBREAK_THRESHOLD);
  }

  public Command intake() {
    return startEnd(() -> roller.runVelocity(-TraderConstants.RPS), roller::stop);
  }

  public Command eject() {
    return startEnd(() -> roller.runVelocity(TraderConstants.RPS), roller::stop);
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }
}
