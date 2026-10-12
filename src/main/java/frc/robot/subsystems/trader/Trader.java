// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trader;

import au.grapplerobotics.interfaces.LaserCanInterface;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.robot.util.sim.SimulationManager;
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
                  CANBus.roboRIO(),
                  Constants.CANConstants.TRADER_BELTS,
                  TraderConstants.MOTOR_CONFIG)
              .addFollower(Constants.CANConstants.TRADER_ROLLER, MotorAlignmentValue.Opposed)
              .setSignalUpdateFrequency(50.0, 50.0)
              .build();
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(1, TraderConstants.MOI, 0, 0, 0, 0),
              0.5,
              0,
              1);
          case REPLAY -> new RollerIO() {};
        };
    roller = new Roller("Trader", leftIO, () -> true);

    beambreak =
        switch (Constants.currentMode) {
          case REAL -> new LaserCanIOReal(
              Constants.CANConstants.TRADER_LASERCAN,
              LaserCanInterface.RangingMode.SHORT,
              LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS,
              new LaserCanInterface.RegionOfInterest(8, 8, 4, 4));
          case SIM -> LaserCanIO.beambreakSim(
              () -> SimulationManager.getInstance().isTraderLoaded(),
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
                && beambreakInputs.distanceMillimeters <= TraderConstants.BEAMBREAK_THRESHOLD);
  }

  public Command eject() {
    return startEnd(() -> roller.runVelocity(TraderConstants.RPS), roller::stop);
  }

  public Command reverse() {
    return startEnd(() -> roller.runVelocity(-TraderConstants.RPS), roller::stop);
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }
}
