package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SimulationHelper {
  public static final Translation3d ELEVATOR_TO_CARROT = new Translation3d(/* TODO find this */ );

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

  @Getter private boolean outtakeLoaded = false;

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
    Pose3d[] carrotPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Carrot");

    Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);

    // Publish ComponentPositions
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPosition",
        new Pose3d(0, 0, elevator.getPositionMeters(), Rotation3d.kZero));

    // to set up the model
    Logger.recordOutput("FieldSimulation/Carrots", carrotPoses);

    if (!outtakeLoaded
        && getNumHeldCarrots() > 0
        && trader.getVelocityRPS() > 50
        && elevator.getSetpoint() == Elevator.Setpoint.STOWED) {
      outtakeLoaded = true;
      Logger.recordOutput("FieldSimulation/IsOuttakeLoaded", outtakeLoaded);
    }
  }

  /** Returns the total number of carrots in the robot. */
  @AutoLogOutput(key = "FieldSimulation/NumCarrotsInBot")
  public int getNumHeldCarrots() {
    return gripperIntake.getGamePiecesAmount();
  }

  /** Returns the number of carrots in the robot's hopper (excludes carrot in outtake). */
  public int getNumCarrotsInHopper() {
    return outtakeLoaded ? getNumHeldCarrots() - 1 : getNumHeldCarrots();
  }

  /** Returns whether there is a game piece in the robot at all */
  public boolean hasAnyGamePiece() {
    return gripperIntake.getGamePiecesAmount() > 0;
  }

  public void score() {
    if (!outtakeLoaded) {
      return;
    }

    gripperIntake.obtainGamePieceFromIntake();
    outtakeLoaded = false;

    Pose3d globalPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    Translation3d elevatorTranslation = new Translation3d(0, 0, elevator.getPositionMeters());
    Translation3d carrotTranslation =
        globalPose.getTranslation().plus(elevatorTranslation).plus(ELEVATOR_TO_CARROT);

    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(carrotTranslation.getX(), carrotTranslation.getY()),
            chassisSpeeds.get(),
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(carrotTranslation.getZ()),
            MetersPerSecond.of(1),
            Degrees.of(0 /* TODO put outtake angle here (static angle) */));

    carrotOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
    carrotOnFly.withProjectileTrajectoryDisplayCallBack(
        hitTrajectory ->
            Logger.recordOutput(
                "FieldSimulation/CarrotHitTrajectory", hitTrajectory.toArray(new Pose3d[0])),
        missTrajectory ->
            Logger.recordOutput(
                "FieldSimulation/CarrotMissTrajectory", missTrajectory.toArray(new Pose3d[0])));
    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }

  public void dropCarrot(HarvestHavocCarrotOnFly.CarrotStations side) {
    HarvestHavocCarrotOnFly carrotOnFly = HarvestHavocCarrotOnFly.dropFromCarrotStation(side);
    SimulatedArena.getInstance().addGamePieceProjectile(carrotOnFly);
  }
}
