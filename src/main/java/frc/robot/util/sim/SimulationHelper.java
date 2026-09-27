package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.util.io.sensors.lasercan.LaserCanIO;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulationHelper {
  public static final Translation3d ELEVATOR_TO_CARROT = new Translation3d(/* TODO find this */ );
  public static final Translation3d ROBOT_TO_TRADER = new Translation3d(/* TODO find this */ );

  @Getter private static SimulationHelper instance;

  public static SimulationHelper createInstance(
      Elevator elevator,
      Trader trader,
      Gripper gripper,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    instance = new SimulationHelper(elevator, trader, gripper, driveSimulation, chassisSpeeds);
    return instance;
  }

  private final Elevator elevator;
  private final Trader trader;
  private final Gripper gripper;
  private final SwerveDriveSimulation driveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;
  private final IntakeSimulation gripperIntake;
  private final IntakeSimulation traderIntake;

  private Pose3d carrotInGripper;
  private Pose3d carrotInTrader;

  private SimulationHelper(
      Elevator elevator,
      Trader trader,
      Gripper gripper,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.trader = trader;
    this.gripper = gripper;
    this.driveSimulation = driveSimulation;
    this.chassisSpeeds = chassisSpeeds;

    gripperIntake =
        IntakeSimulation.InTheFrameIntake(
            "Carrot", driveSimulation, Meters.of(0.5), IntakeSimulation.IntakeSide.FRONT, 1);
    traderIntake =
        IntakeSimulation.InTheFrameIntake(
            "Carrot", driveSimulation, Meters.of(0.7), IntakeSimulation.IntakeSide.BACK, 3);
  }

  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    Pose3d robotPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/RobotPosition", robotPose.toPose2d());
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPosition",
        new Pose3d(0, 0, elevator.getPositionMeters(), Rotation3d.kZero));

    Pose3d[] carrotPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Carrot");
    Logger.recordOutput("FieldSimulation/Carrots", carrotPoses);
    // gripper carrot
    if (isGripperLoaded()) {
      Translation3d elevatorTranslation = new Translation3d(0, 0, elevator.getPositionMeters());
      carrotInGripper =
          new Pose3d(
              robotPose.getTranslation().plus(elevatorTranslation).plus(ELEVATOR_TO_CARROT),
              Rotation3d.kZero);
      Logger.recordOutput("FieldSimulation/HeldCarrots/Gripper",  carrotInGripper);
    }
    else {
      Logger.recordOutput("FieldSimulation/HeldCarrots/Gripper", new Pose3d());
    }
    // trader carrot
    if (isTraderLoaded()) {
      carrotInTrader =
          new Pose3d(
              robotPose.getTranslation().plus(ROBOT_TO_TRADER),
              Rotation3d.kZero);
      Logger.recordOutput("FieldSimulation/HeldCarrots/Trader", carrotInTrader);
    }
    else {
      Logger.recordOutput("FieldSimulation/HeldCarrots/Trader", new Pose3d());
    }
  }

  private boolean isGripperLoaded() {
    return gripperIntake.getGamePiecesAmount() > 0;
  }

  private boolean isTraderLoaded() {
    return traderIntake.getGamePiecesAmount() > 0;
  }

  public LaserCanIO getGripperLaserCan(double threshold) {
    return inputs -> {
      inputs.connected = true;
      inputs.measurementValid = true;
      inputs.distanceMillimeters = isGripperLoaded() ? threshold - 1 : threshold + 1;
    };
  }

  public LaserCanIO getTraderLaserCan(double threshold) {
    return inputs -> {
      inputs.connected = true;
      inputs.measurementValid = true;
      inputs.distanceMillimeters = isTraderLoaded() ? threshold - 1 : threshold + 1;
    };
  }

  public void gripperScore() {
    if (!gripperIntake.obtainGamePieceFromIntake()) return;

    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(carrotInGripper.getX(), carrotInGripper.getY()),
            chassisSpeeds.get(),
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(carrotInGripper.getZ()),
            MetersPerSecond.of(1),
            Degrees.of(0 /* TODO put outtake angle here (static angle) */));

    carrotOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }

  public void traderScore() {
    if (!traderIntake.obtainGamePieceFromIntake()) return;

    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(carrotInTrader.getX(), carrotInTrader.getY()),
            chassisSpeeds.get(),
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(carrotInTrader.getZ()),
            MetersPerSecond.of(1),
            Degrees.of(0 /* TODO put outtake angle here (static angle) */));

    carrotOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }

  public void dropCarrot(HarvestHavocCarrotOnFly.CarrotStations side) {
    HarvestHavocCarrotOnFly carrotOnFly = HarvestHavocCarrotOnFly.dropFromCarrotStation(side);
    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }
}
