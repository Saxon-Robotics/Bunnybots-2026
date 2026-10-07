package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.util.AutoAlign;
import frc.robot.util.RobotUtil;
import frc.robot.util.sim.HarvestHavocCarrotOnFly;
import frc.robot.util.sim.SimulationHelper;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class BLineAutos {
  private static FollowPath.Builder builder;

  private static Drive drive;
  private static Elevator elevator;
  private static Trader trader;
  private static Gripper gripper;

  private static Command traderEject;
  private static Command rampIntake;
  private static Command scoreL2;

  public static void configure(Drive drive, Elevator elevator, Trader trader, Gripper gripper) {
    BLineAutos.drive = drive;
    BLineAutos.elevator = elevator;
    BLineAutos.trader = trader;
    BLineAutos.gripper = gripper;

    builder =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(2.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.4),
                new PIDController(0.2, 0.0, 0.0))
            .withDefaultShouldFlip()
            // in this field, alliance sides are mirrored from each other
            .withShouldMirror(RobotUtil::isRedAlliance)
            .withTRatioBasedTranslationHandoffs(true);

    FollowPath.setPoseLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    //    FollowPath.setTranslationListLoggingConsumer(
    //        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));

    Command ovenAlign =
        Commands.runOnce(() -> AutoAlign.setTargetPose(AutoAlign.Target.OVEN, drive.getPose()));
    Command rampAlign =
        Commands.runOnce(() -> AutoAlign.setTargetPose(AutoAlign.Target.RAMP, drive.getPose()));
    Command pantryAlign =
        Commands.runOnce(() -> AutoAlign.setTargetPose(AutoAlign.Target.PANTRY, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));

    traderEject = Commands.sequence(ovenAlign, trader.eject().until(() -> !trader.isLoaded()));
    rampIntake = Commands.sequence(rampAlign, gripper.intake().until(gripper::isLoaded));
    if (Constants.currentMode == Constants.Mode.SIM) {
      rampIntake =
          rampIntake.alongWith(
              Commands.runOnce(
                  () ->
                      SimulationHelper.getInstance()
                          .dropCarrot(
                              RobotUtil.isRedAlliance()
                                  ? HarvestHavocCarrotOnFly.CarrotStations.RED_RAMP
                                  : HarvestHavocCarrotOnFly.CarrotStations.BLUE_RAMP)));
    }
    scoreL2 = Commands.sequence(pantryAlign, gripper.eject().until(() -> !gripper.isLoaded()));
  }

  public static Command carrotCake() {
    Path ovenToRampPath = new Path("OvenToRamp");
    Path rampToPantryPath = new Path("RampToPantry");
    Path pantryToWaitPath = new Path("PantryToWait");

    Command ovenToRamp =
        Commands.defer(
            () -> builder.withPoseReset(drive::setPose).build(ovenToRampPath), Set.of(drive));
    Command rampToPantry =
        Commands.defer(
            () -> builder.withPoseReset(ignored -> {}).build(rampToPantryPath), Set.of(drive));
    Command pantryToWait = Commands.defer(() -> builder.build(pantryToWaitPath), Set.of(drive));

    return BLineCommands.sequence(
        traderEject, ovenToRamp, rampIntake, rampToPantry, scoreL2, pantryToWait);
  }

  public static Command ovenDump() {
    Command ovenToWait =
        Commands.defer(
            () -> builder.withPoseReset(drive::setPose).build(new Path("OvenToWait")),
            Set.of(drive));

    return BLineCommands.sequence(traderEject, ovenToWait);
  }
}
