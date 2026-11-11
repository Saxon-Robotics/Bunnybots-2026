package frc.robot.commands;

import static frc.robot.commands.SelfDrivingConstants.*;
import static frc.robot.util.AutoAlign.Target.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.trader.Trader;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlign.Target;
import frc.robot.util.DecisionTree;
import frc.robot.util.LoggedTracer;
import frc.robot.util.RobotUtil;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SelfDriving {
  private DecisionTree.Node tree = DecisionTree.EMPTY;

  private final Drive drive;
  private final Elevator elevator;
  private final BooleanSupplier timeToPark;
  private final LoggedNetworkBoolean holdForDinner =
      new LoggedNetworkBoolean("/SmartDashboard/SelfDriving/HoldForDinner", false);

  public SelfDriving(
      ObjectiveTracker tracker,
      Drive drive,
      Elevator elevator,
      Arm arm,
      Gripper gripper,
      Trader trader) {
    this.drive = drive;
    this.elevator = elevator;

    Command scoreL1 = scorePantry(0, tracker, drive, elevator, arm, gripper).withName("Score L1");
    Command scoreL2 = scorePantry(1, tracker, drive, elevator, arm, gripper).withName("Score L2");
    Command scoreL3 = scorePantry(2, tracker, drive, elevator, arm, gripper).withName("Score L3");
    Command scoreOven =
        driveAndAlign(drive, OVEN)
            .andThen(
                trader
                    .eject()
                    .until(() -> !trader.isLoaded())
                    .withTimeout(OVEN_EJECT_TIMEOUT_SECONDS))
            .andThen(Commands.waitSeconds(OVEN_CREDIT_SECONDS))
            .withName("Score Oven");
    Command collectRamp =
        driveAndAlign(drive, RAMP)
            .andThen(
                SuperstructureCommands.intake(arm, gripper)
                    .until(gripper::isLoaded)
                    .withTimeout(RAMP_INTAKE_TIMEOUT_SECONDS))
            .deadlineFor(elevator.ramp())
            .andThen(
                Commands.runOnce(
                    () -> {
                      if (!gripper.isLoaded()) tracker.forfeitCake();
                    }))
            .andThen(elevator.stow())
            .beforeStarting(() -> tracker.setCollectingCake(true))
            .finallyDo(() -> tracker.setCollectingCake(false))
            .withName("Collect Ramp");
    Command collectGround =
        SuperstructureCommands.intakeFromGround(elevator, arm, gripper)
            .until(gripper::isLoaded)
            .withName("Collect Ground");
    Command traderCollectDepot =
        driveAndAlign(drive, DEPOT)
            .andThen(waitForPieceLimit(gripper, trader))
            .withName("Collect Depot - Trader");
    Command elevatorCollectDepot =
        gripperDepotIntake(drive, elevator, gripper, trader).withName("Collect Depot - Elevator");
    Command collectDepotMixed =
        gripperDepotIntake(drive, elevator, gripper, trader)
            .andThen(waitForPieceLimit(gripper, trader))
            .withName("Collect Depot - Mixed");
    Command stageCenter =
        drive
            .defer(() -> drive.pathfindToPose(stagePose()))
            .andThen(Commands.run(drive::stop, drive).onlyWhile(DriverStation::isAutonomousEnabled))
            .withName("Stage Center");
    Command park =
        driveAndAlign(drive, TABLE)
            .alongWith(elevator.stow())
            .andThen(Commands.run(drive::stop, drive))
            .withName("Park");

    DecisionTree.registerCommand("scoreL1", scoreL1);
    DecisionTree.registerCommand("scoreL2", scoreL2);
    DecisionTree.registerCommand("scoreL3", scoreL3);
    DecisionTree.registerCommand("scoreOven", scoreOven);
    DecisionTree.registerCommand("collectRamp", collectRamp);
    DecisionTree.registerCommand("collectGround", collectGround);
    DecisionTree.registerCommand("traderCollectDepot", traderCollectDepot);
    DecisionTree.registerCommand("elevatorCollectDepot", elevatorCollectDepot);
    DecisionTree.registerCommand("collectDepotMixed", collectDepotMixed);
    DecisionTree.registerCommand("stageCenter", stageCenter);
    DecisionTree.registerCommand("park", park);

    DecisionTree.registerCondition("gripperLoaded", gripper::isLoaded);
    DecisionTree.registerCondition(
        "gripperHasCake", () -> tracker.getGripperHeld() == GamePiece.CARROT_CAKE);
    DecisionTree.registerCondition("traderLoaded", trader::isLoaded);
    DecisionTree.registerCondition("traderFull", trader::isFull);
    DecisionTree.registerCondition(
        "atPieceLimit", () -> heldCount(gripper, trader) >= MAX_GAME_PIECES);

    DecisionTree.registerCondition("cakeReady", tracker::isCakeReady);
    DecisionTree.registerCondition("trackingRP", tracker::isTrackingRP);
    DecisionTree.registerCondition("stockedUp", tracker::isStockedUp);
    DecisionTree.registerCondition("bakedUp", tracker::isBakedUp);

    DecisionTree.registerCondition("pantryL1Open", () -> tracker.hasPantrySlot(0));
    DecisionTree.registerCondition("pantryL2Open", () -> tracker.hasPantrySlot(1));
    DecisionTree.registerCondition("pantryL3Open", () -> tracker.hasPantrySlot(2));
    DecisionTree.registerCondition("anyPantryOpen", tracker::hasAnyPantrySlot);
    DecisionTree.registerCondition(
        "pantryNeedsStock",
        () ->
            tracker.levelNeedsStock(0) || tracker.levelNeedsStock(1) || tracker.levelNeedsStock(2));
    DecisionTree.registerCondition("pantryL1NeedsCake", () -> tracker.levelNeedsCake(0));
    DecisionTree.registerCondition("pantryL2NeedsCake", () -> tracker.levelNeedsCake(1));
    DecisionTree.registerCondition("pantryL3NeedsCake", () -> tracker.levelNeedsCake(2));
    DecisionTree.registerCondition("pantryL1NeedsStock", () -> tracker.levelNeedsStock(0));
    DecisionTree.registerCondition("pantryL2NeedsStock", () -> tracker.levelNeedsStock(1));
    DecisionTree.registerCondition("pantryL3NeedsStock", () -> tracker.levelNeedsStock(2));
    DecisionTree.registerCondition("pantryL1CarrotOk", () -> tracker.levelOpenForCarrot(0));
    DecisionTree.registerCondition("pantryL2CarrotOk", () -> tracker.levelOpenForCarrot(1));
    DecisionTree.registerCondition("pantryL3CarrotOk", () -> tracker.levelOpenForCarrot(2));

    timeToPark =
        () -> {
          double matchTime = DriverStation.getMatchTime();
          return DriverStation.isTeleopEnabled()
              && matchTime >= 0.0
              && matchTime <= secondsToReach(TABLE) + PARK_MARGIN_SECONDS;
        };
    DecisionTree.registerCondition("isAuto", DriverStation::isAutonomous);
    DecisionTree.registerCondition("isEndgame", SelfDriving::isEndgame);
    DecisionTree.registerCondition(
        "holdForDinner", () -> holdForDinner.get() && tracker.isTrackingRP() && isEndgame());
    DecisionTree.registerCondition("timeToPark", timeToPark);
    DecisionTree.registerCondition(
        "canScorePantryBeforePark", () -> hasTimeBeforePark(PANTRY, PANTRY_SCORE_SECONDS));
    DecisionTree.registerCondition(
        "canScoreOvenBeforePark", () -> hasTimeBeforePark(OVEN, OVEN_SCORE_SECONDS));
    DecisionTree.registerCondition(
        "canCollectRampBeforePark", () -> hasTimeBeforePark(RAMP, RAMP_COLLECT_SECONDS));
    DecisionTree.registerCondition(
        "canCollectDepotBeforePark", () -> hasTimeBeforePark(DEPOT, DEPOT_COLLECT_SECONDS));
  }

  private static boolean isEndgame() {
    double matchTime = DriverStation.getMatchTime();
    return DriverStation.isTeleopEnabled() && matchTime >= 0.0 && matchTime <= ENDGAME_SECONDS;
  }

  private static int heldCount(Gripper gripper, Trader trader) {
    return trader.getNumGamePieces() + (gripper.isLoaded() ? 1 : 0);
  }

  private static Command waitForPieceLimit(Gripper gripper, Trader trader) {
    return Commands.waitUntil(() -> heldCount(gripper, trader) >= MAX_GAME_PIECES)
        .withTimeout(DEPOT_WAIT_TIMEOUT_SECONDS);
  }

  private Command gripperDepotIntake(
      Drive drive, Elevator elevator, Gripper gripper, Trader trader) {
    return driveAndAlign(drive, DEPOT)
        .andThen(
            gripper
                .intake()
                .until(() -> gripper.isLoaded() || trader.getNumGamePieces() >= MAX_GAME_PIECES - 1)
                .withTimeout(DEPOT_WAIT_TIMEOUT_SECONDS))
        .deadlineFor(elevator.ramp())
        .andThen(elevator.stow());
  }

  private Pose2d stagePose() {
    boolean isRed = RobotUtil.isRedAlliance();
    Pose2d depot = REAR_DEPOT.getPose(isRed);
    double x =
        FieldConstants.ORIGIN.getX()
            + (isRed ? STAGE_DISTANCE_FROM_CENTER_METERS : -STAGE_DISTANCE_FROM_CENTER_METERS);
    return new Pose2d(x, depot.getY(), depot.getRotation());
  }

  private static double travelSeconds(Translation2d from, Translation2d to) {
    return from.getDistance(to) / AVERAGE_SPEED_METERS_PER_SEC + ALIGN_SECONDS;
  }

  private static Translation2d resolvedTranslation(Target target, Pose2d robot, boolean isRed) {
    return AutoAlign.resolve(target, robot, isRed).getPose(isRed).getTranslation();
  }

  private double secondsToReach(Target target) {
    Pose2d robot = drive.getPose();
    return travelSeconds(
        robot.getTranslation(), resolvedTranslation(target, robot, RobotUtil.isRedAlliance()));
  }

  private boolean hasTimeBeforePark(Target target, double taskSeconds) {
    double matchTime = DriverStation.getMatchTime();
    if (!DriverStation.isTeleopEnabled() || matchTime < 0.0) return true;
    boolean isRed = RobotUtil.isRedAlliance();
    Pose2d robot = drive.getPose();
    Translation2d goal = resolvedTranslation(target, robot, isRed);
    double needed =
        travelSeconds(robot.getTranslation(), goal)
            + taskSeconds
            + travelSeconds(goal, TABLE.getPose(isRed).getTranslation())
            + PARK_MARGIN_SECONDS;
    return matchTime > needed;
  }

  private Command driveAndAlign(Drive drive, Target target) {
    return drive
        .defer(() -> drive.pathfindToPose(AutoAlign.setTarget(target, drive.getPose())))
        .andThen(
            DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget)
                .withTimeout(ALIGN_TIMEOUT_SECONDS));
  }

  private Command driveAndAlign(Drive drive, Target target, Pose2d pose) {
    return drive
        .pathfindToPose(pose)
        .andThen(() -> AutoAlign.setTargetPose(target, pose))
        .andThen(
            DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget)
                .withTimeout(ALIGN_TIMEOUT_SECONDS));
  }

  private Command scorePantry(
      int level,
      ObjectiveTracker tracker,
      Drive drive,
      Elevator elevator,
      Arm arm,
      Gripper gripper) {
    if (level < 0 || level > 2) {
      throw new IllegalArgumentException("Invalid pantry level: " + level);
    }
    return Commands.defer(
        () -> {
          Pose2d pose = tracker.getPantryPose(level, drive.getPose());
          if (pose == null) return Commands.none();
          Command raiseElevator =
              switch (level) {
                case 0 -> elevator.l1();
                case 1 -> elevator.l2();
                default -> elevator.l3();
              };
          return driveAndAlign(drive, PANTRY, pose)
              .andThen(raiseElevator.withTimeout(ELEVATOR_TIMEOUT_SECONDS))
              .andThen(
                  Commands.deadline(
                      Commands.waitUntil(arm::hasReachedSetpoint)
                          .withTimeout(ARM_TIMEOUT_SECONDS)
                          .andThen(gripper.eject().withTimeout(GripperConstants.OUTTAKE_TIMEOUT)),
                      arm.pantry()))
              .andThen(elevator.stow());
        },
        Set.of(drive, elevator, arm, gripper));
  }

  public Command routine() {
    boolean[] pickedWhileParking = {false};
    return Commands.deferredProxy(
            () -> {
              pickedWhileParking[0] = timeToPark.getAsBoolean();
              List<String> traversalPath = new ArrayList<>();
              Command next = tree.evaluate(traversalPath);
              Logger.recordOutput(
                  "SelfDriving/TraversalPath", traversalPath.toArray(new String[0]));
              LoggedTracer.record("SelfDriving/Decision");
              return next;
            })
        .until(() -> !pickedWhileParking[0] && timeToPark.getAsBoolean())
        .repeatedly()
        .finallyDo(() -> CommandScheduler.getInstance().schedule(elevator.stow()));
  }

  public void setUpChooser(LoggedDashboardChooser<File> chooser) {
    chooser.addDefaultOption("None", null);
    for (File file : DecisionTree.getAllStrategyFiles()) {
      String fileName = file.getName();
      chooser.addOption(fileName.substring(0, fileName.lastIndexOf(".")), file);
    }
    chooser.onChange(this::changeStrategy);
    changeStrategy(chooser.get());
  }

  public void changeStrategy(File file) {
    if (file == null) {
      tree = DecisionTree.EMPTY;
      return;
    }
    try {
      tree = DecisionTree.fromJson(file);
    } catch (IOException e) {
      System.err.println("Strategy not found: " + file.getName());
      e.printStackTrace();
      tree = DecisionTree.EMPTY;
    }
  }
}
