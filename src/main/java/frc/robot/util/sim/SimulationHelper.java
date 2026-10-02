package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.trader.TraderConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import lombok.Getter;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulationHelper {
  private static final Translation3d ELEVATOR_TO_CARROT = new Translation3d(0.3, 0.08, 0.425);
  private static final Translation3d[] TRADER_SLOTS =
      new Translation3d[] {
        new Translation3d(-0.3, 0.1, 0.31),
        new Translation3d(-0.1, 0.1, 0.35),
        new Translation3d(-0.18, 0.1, 0.51)
      };

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
  private Pose3d[] carrotsInTrader = new Pose3d[3];

  private GamePieceOnFieldSimulation lastCarrot;

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

    Rectangle gripperShape = new Rectangle(0.45, 0.7);
    Rectangle traderShape = new Rectangle(0.3, 0.7);
    gripperShape.translate(gripperShape.getWidth() / 2, 0);
    traderShape.translate(-traderShape.getWidth() / 2, 0);

    gripperIntake = new IntakeSimulation("Carrot", driveSimulation, gripperShape, 1);
    traderIntake = new IntakeSimulation("Carrot", driveSimulation, traderShape, 3);
    // dedup to prevent double count
    gripperIntake.setCustomIntakeCondition(this::checkIntakeCarrot);
    traderIntake.setCustomIntakeCondition(this::checkIntakeCarrot);
    traderIntake.startIntake();

    // outtake activations
    new Trigger(() -> gripper.getVelocityRPS() > 35).onTrue(Commands.runOnce(this::gripperScore));
    new Trigger(() -> trader.getVelocityRPS() > 35)
        .whileTrue(
            Commands.repeatingSequence(
                Commands.runOnce(this::traderScore),
                Commands.waitSeconds(TraderConstants.CARROTS_PER_SEC)));
  }

  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    Pose3d robotPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/RobotPosition", robotPose.toPose2d());
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPosition",
        new Pose3d(0, 0, elevator.getPositionMeters(), Rotation3d.kZero));
    Logger.recordOutput(
        "FieldSimulation/IntakeSimulation/GripperCount", gripperIntake.getGamePiecesAmount());
    Logger.recordOutput(
        "FieldSimulation/IntakeSimulation/TraderCount", traderIntake.getGamePiecesAmount());

    Pose3d[] carrotPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Carrot");
    Logger.recordOutput("FieldSimulation/Carrots", carrotPoses);

    // intake activation
    if (gripper.getVelocityRPS() < -35) {
      gripperIntake.startIntake();
    } else {
      gripperIntake.stopIntake();
    }

    Rotation3d carrotRotation =
        robotPose.getRotation().rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(90)));
    // gripper carrot
    if (isGripperLoaded()) {
      Translation3d elevatorTranslation = new Translation3d(0, 0, elevator.getPositionMeters());
      carrotInGripper =
          new Pose3d(
              robotPose
                  .getTranslation()
                  .plus(elevatorTranslation)
                  .plus(ELEVATOR_TO_CARROT.rotateBy(robotPose.getRotation())),
              carrotRotation);
      Logger.recordOutput("FieldSimulation/HeldCarrots/Gripper", carrotInGripper);
    } else {
      Logger.recordOutput("FieldSimulation/HeldCarrots/Gripper", Pose3d.kZero);
    }
    // trader carrots
    Arrays.fill(carrotsInTrader, Pose3d.kZero);
    for (int i = 0; i < traderIntake.getGamePiecesAmount(); i++) {
      carrotsInTrader[i] =
          new Pose3d(
              robotPose.getTranslation().plus(TRADER_SLOTS[i].rotateBy(robotPose.getRotation())),
              carrotRotation);
    }
    Logger.recordOutput("FieldSimulation/HeldCarrots/Trader", carrotsInTrader);
  }

  private boolean checkIntakeCarrot(GamePieceOnFieldSimulation carrot) {
    if (carrot == lastCarrot) return false;
    lastCarrot = carrot;
    return true;
  }

  public boolean isGripperLoaded() {
    return gripperIntake.getGamePiecesAmount() > 0;
  }

  public boolean isTraderLoaded() {
    return traderIntake.getGamePiecesAmount() > 0;
  }

  private void gripperScore() {
    if (!gripperIntake.obtainGamePieceFromIntake()) return;

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            new Translation2d(carrotInGripper.getMeasureX(), carrotInGripper.getMeasureY()),
            new Translation2d(1, robotPose.getRotation()),
            carrotInGripper.getZ(),
            1.0,
            new Rotation3d(robotPose.getRotation().plus(Rotation2d.kCCW_90deg)));

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

  private void traderScore() {
    if (!traderIntake.obtainGamePieceFromIntake()) return;

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            new Translation2d(carrotsInTrader[0].getX(), carrotsInTrader[0].getY()),
            new Translation2d(-1.5, robotPose.getRotation()),
            carrotsInTrader[0].getZ(),
            1.0,
            new Rotation3d(robotPose.getRotation().plus(Rotation2d.kCCW_90deg)));

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
