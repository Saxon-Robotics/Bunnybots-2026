package frc.robot.util.sim;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.trader.TraderConstants;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.function.Supplier;
import lombok.Getter;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {
  private static final String CARROT = HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO.type();
  private static final String CAKE = HarvestHavocCarrotOnField.HARVEST_HAVOC_CAKE_INFO.type();

  private static final Translation3d ELEVATOR_TO_CARROT = new Translation3d(0.3, 0.08, 0.425);
  private static final Translation3d[] TRADER_SLOTS =
      new Translation3d[] {
        new Translation3d(-0.3, 0.1, 0.31),
        new Translation3d(-0.1, 0.1, 0.35),
        new Translation3d(-0.18, 0.1, 0.51)
      };

  @Getter private static SimulationManager instance;

  public static SimulationManager createInstance(
      Elevator elevator,
      Trader trader,
      Gripper gripper,
      SwerveDriveSimulation driveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    instance = new SimulationManager(elevator, trader, gripper, driveSimulation, chassisSpeeds);
    return instance;
  }

  private final Elevator elevator;
  private final Trader trader;
  private final Gripper gripper;
  private final SwerveDriveSimulation driveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;
  private final IntakeSimulation gripperIntake;
  private final IntakeSimulation gripperCakeIntake;
  private final IntakeSimulation traderIntake;
  private final IntakeSimulation traderCakeIntake;

  private Pose3d carrotInGripper;
  private Pose3d[] gamePiecesInTrader = new Pose3d[3];
  private Pose3d[] heldCarrotsInTrader = new Pose3d[3];
  private Pose3d[] heldCakesInTrader = new Pose3d[3];
  private ArrayDeque<Boolean> traderOrder = new ArrayDeque<>();

  private GamePieceOnFieldSimulation lastCarrot;

  private int traderCountBeforePreload;
  private int traderCakeCountBeforePreload;
  private ArrayDeque<Boolean> traderOrderBeforePreload = new ArrayDeque<>();

  private SimulationManager(
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

    gripperIntake = new IntakeSimulation(CARROT, driveSimulation, gripperShape, 1);
    gripperCakeIntake = new IntakeSimulation(CAKE, driveSimulation, gripperShape, 1);
    traderIntake = new IntakeSimulation(CARROT, driveSimulation, traderShape, 3);
    traderCakeIntake = new IntakeSimulation(CAKE, driveSimulation, traderShape, 3);

    gripperIntake.setCustomIntakeCondition(p -> checkIntakeCarrot(p) && getGripperCount() < 1);
    gripperCakeIntake.setCustomIntakeCondition(p -> checkIntakeCarrot(p) && getGripperCount() < 1);
    traderIntake.setCustomIntakeCondition(p -> traderAccept(p, false));
    traderCakeIntake.setCustomIntakeCondition(p -> traderAccept(p, true));
    traderIntake.startIntake();
    traderCakeIntake.startIntake();

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
    Logger.recordOutput("FieldSimulation/IntakeSimulation/GripperCount", getGripperCount());
    Logger.recordOutput("FieldSimulation/IntakeSimulation/TraderCount", getTraderCount());

    Pose3d[] carrotPoses = SimulatedArena.getInstance().getGamePiecesArrayByType(CARROT);
    Logger.recordOutput("FieldSimulation/Carrots", carrotPoses);
    Pose3d[] cakePoses = SimulatedArena.getInstance().getGamePiecesArrayByType(CAKE);
    Logger.recordOutput("FieldSimulation/Cakes", cakePoses);

    // intake activation
    if (gripper.getVelocityRPS() < -35) {
      gripperIntake.startIntake();
      gripperCakeIntake.startIntake();
    } else {
      gripperIntake.stopIntake();
      gripperCakeIntake.stopIntake();
    }

    Rotation3d carrotRotation =
        robotPose.getRotation().rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(90)));

    if (isGripperLoaded()) {
      Translation3d elevatorTranslation = new Translation3d(0, 0, elevator.getPositionMeters());
      carrotInGripper =
          new Pose3d(
              robotPose
                  .getTranslation()
                  .plus(elevatorTranslation)
                  .plus(ELEVATOR_TO_CARROT.rotateBy(robotPose.getRotation())),
              carrotRotation);
      boolean holdingCake = gripperCakeIntake.getGamePiecesAmount() > 0;
      Logger.recordOutput(
          "FieldSimulation/HeldCarrots/Gripper", holdingCake ? Pose3d.kZero : carrotInGripper);
      Logger.recordOutput(
          "FieldSimulation/HeldCakes/Gripper", holdingCake ? carrotInGripper : Pose3d.kZero);
    } else {
      Logger.recordOutput("FieldSimulation/HeldCarrots/Gripper", Pose3d.kZero);
      Logger.recordOutput("FieldSimulation/HeldCakes/Gripper", Pose3d.kZero);
    }

    Arrays.fill(gamePiecesInTrader, Pose3d.kZero);
    Arrays.fill(heldCarrotsInTrader, Pose3d.kZero);
    Arrays.fill(heldCakesInTrader, Pose3d.kZero);
    int i = 0;
    for (boolean isCake : traderOrder) {
      gamePiecesInTrader[i] =
          new Pose3d(
              robotPose.getTranslation().plus(TRADER_SLOTS[i].rotateBy(robotPose.getRotation())),
              carrotRotation);
      if (isCake) {
        heldCakesInTrader[i] = gamePiecesInTrader[i];
      } else {
        heldCarrotsInTrader[i] = gamePiecesInTrader[i];
      }
      i++;
    }
    Logger.recordOutput("FieldSimulation/HeldCarrots/Trader", heldCarrotsInTrader);
    Logger.recordOutput("FieldSimulation/HeldCakes/Trader", heldCakesInTrader);
  }

  private boolean checkIntakeCarrot(GamePieceOnFieldSimulation carrot) {
    if (carrot == lastCarrot) return false;
    lastCarrot = carrot;
    return true;
  }

  private boolean traderAccept(GamePieceOnFieldSimulation piece, boolean isCake) {
    if (getTraderCount() >= 3 || !checkIntakeCarrot(piece)) return false;
    traderOrder.addLast(isCake);
    return true;
  }

  private int getGripperCount() {
    return gripperIntake.getGamePiecesAmount() + gripperCakeIntake.getGamePiecesAmount();
  }

  private int getTraderCount() {
    return traderIntake.getGamePiecesAmount() + traderCakeIntake.getGamePiecesAmount();
  }

  public boolean isGripperLoaded() {
    return getGripperCount() > 0;
  }

  public boolean isTraderLoaded() {
    return getTraderCount() > 0;
  }

  public void preloadTrader() {
    traderCountBeforePreload = traderIntake.getGamePiecesAmount();
    traderCakeCountBeforePreload = traderCakeIntake.getGamePiecesAmount();
    traderOrderBeforePreload = new ArrayDeque<>(traderOrder);
    traderIntake.setGamePiecesCount(2);
    traderCakeIntake.setGamePiecesCount(0);
    traderOrder.clear();
    traderOrder.addLast(false);
    traderOrder.addLast(false);
  }

  public void undoTraderPreload() {
    traderIntake.setGamePiecesCount(traderCountBeforePreload);
    traderCakeIntake.setGamePiecesCount(traderCakeCountBeforePreload);
    traderOrder = new ArrayDeque<>(traderOrderBeforePreload);
  }

  private void gripperScore() {
    boolean isCake;
    if (gripperIntake.obtainGamePieceFromIntake()) {
      isCake = false;
    } else if (gripperCakeIntake.obtainGamePieceFromIntake()) {
      isCake = true;
    } else {
      return;
    }

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            isCake,
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
    Boolean isCake = traderOrder.pollFirst();
    if (isCake == null) return;
    IntakeSimulation source = isCake ? traderCakeIntake : traderIntake;
    if (!source.obtainGamePieceFromIntake()) return;

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HarvestHavocCarrotOnFly carrotOnFly =
        new HarvestHavocCarrotOnFly(
            isCake,
            new Translation2d(gamePiecesInTrader[0].getX(), gamePiecesInTrader[0].getY()),
            new Translation2d(-1.5, robotPose.getRotation()),
            gamePiecesInTrader[0].getZ(),
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
