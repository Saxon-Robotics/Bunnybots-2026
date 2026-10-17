package frc.robot.subsystems.objectivetracker;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoAlign;
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

public class ObjectiveTracker extends SubsystemBase {
  public enum GamePiece {
    CARROT,
    CARROT_CAKE
  }

  private static final double NEAR_THRESHOLD = 0.6;

  // tracked subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Gripper gripper;
  private final Trader trader;

  // not gonna track number of pieces because that's annoying
  private GamePiece traderHeld;
  private GamePiece gripperHeld;

  // x positions of each game piece in pantry
  @Getter private final List<List<Double>> pantry = new ArrayList<>(3);

  // whether we've dumped carrots in oven
  private boolean cakeReady;

  // RP tracking
  private int bakedUpProgress;
  @Getter private boolean stockedUp;

  public ObjectiveTracker(
      Drive drive, Vision vision, Elevator elevator, Gripper gripper, Trader trader) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.gripper = gripper;
    this.trader = trader;

    for (int i = 0; i < 3; i++) {
      pantry.add(new ArrayList<>(5));
    }

    // set up triggers
    new Trigger(gripper::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (AutoAlign.getLastTarget() == AutoAlign.Target.RAMP) {
                    gripperHeld = GamePiece.CARROT_CAKE;
                    cakeReady = false;
                  } else {
                    gripperHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  int scoringIndex = elevator.getSetpoint().scoringIndex;
                  if (scoringIndex > -1 && AutoAlign.getLastTarget() == AutoAlign.Target.PANTRY) {
                    pantry.get(scoringIndex).add(drive.getPose().getX());
                  }
                  gripperHeld = null;
                }));

    new Trigger(trader::isLoaded)
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (AutoAlign.getLastTarget() == AutoAlign.Target.RAMP) {
                    traderHeld = GamePiece.CARROT_CAKE;
                  } else {
                    traderHeld = GamePiece.CARROT;
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (drive
                          .getPose()
                          .getTranslation()
                          .getSquaredDistance(
                              AutoAlign.Target.OVEN
                                  .getPose(RobotUtil.isRedAlliance())
                                  .getTranslation())
                      < NEAR_THRESHOLD) {
                    traderHeld = null;
                    cakeReady = true;
                  }
                }));
  }

  @Override
  public void periodic() {
    // RP checks and pantry vision would go here
  }

  public boolean isBakedUp() {
    return bakedUpProgress > 2;
  }
}
