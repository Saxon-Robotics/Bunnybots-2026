// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trader;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.lasercan.LaserCanIO;
import frc.robot.util.io.sensors.lasercan.LaserCanIOInputsAutoLogged;
import frc.robot.util.io.sensors.lasercan.LaserCanIOReal;
import frc.robot.util.sim.SimulationHelper;
import org.littletonrobotics.junction.Logger;

public class Trader extends SubsystemBase {
  private final Roller leftRoller;
  private final Roller rightRoller;
  private final LaserCanIO beambreak;
  private final LaserCanIOInputsAutoLogged beambreakInputs = new LaserCanIOInputsAutoLogged();

  public Trader() {
    RollerIO leftIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.TRADER_LEFT,
                  TraderConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(1, TraderConstants.MOI, 0, 0, 0, 0),
              TraderConstants.KP,
              TraderConstants.KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    RollerIO rightIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.TRADER_RIGHT,
                  TraderConstants.MOTOR_CONFIG)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(1, TraderConstants.MOI, 0, 0, 0, 0),
              TraderConstants.KP,
              TraderConstants.KD,
              0);
          case REPLAY -> new RollerIO() {};
        };
    leftRoller = new Roller("Trader", leftIO);
    rightRoller = new Roller("Trader", rightIO);

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
    leftRoller.periodic();
    rightRoller.periodic();
    beambreak.updateInputs(beambreakInputs);
    Logger.processInputs("Trader/DistanceSensor", beambreakInputs);
  }

  private void runTogether(double rps) {
    leftRoller.runVelocity(rps);
    rightRoller.runVelocity(rps);
  }

  private void runOpposed(double rps) {
    leftRoller.runVelocity(rps);
    rightRoller.runVelocity(-rps);
  }

  private void stop() {
    leftRoller.stop();
    rightRoller.stop();
  }

  public Command intake() {
    return startEnd(() -> runTogether(-TraderConstants.RPS), this::stop);
  }

  public Command sterilize() {
    return startEnd(() -> runOpposed(TraderConstants.STERILIZATION_RPS), this::stop);
  }

  public Command eject() {
    return startEnd(() -> runTogether(TraderConstants.RPS), this::stop);
  }

  public double getVelocityRPS() {
    return leftRoller.getVelocityRPS();
  }

  public boolean hasGamePiece() {
    return beambreakInputs.measurementValid
        && beambreakInputs.distanceMillimeters <= TraderConstants.BEAMBREAK_THRESHOLD;
  }
}
