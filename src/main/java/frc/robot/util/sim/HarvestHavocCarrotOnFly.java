package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class HarvestHavocCarrotOnFly extends GamePieceProjectile {
  public HarvestHavocCarrotOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTouchGroundHeight(0.2);
  }

  public HarvestHavocCarrotOnFly(
      Translation2d initialPosition,
      Translation2d initialLaunchingVelocityMPS,
      double initialHeight,
      double initialVerticalSpeedMPS,
      Rotation3d gamePieceRotation) {
    super(
        HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
        initialPosition,
        initialLaunchingVelocityMPS,
        initialHeight,
        initialVerticalSpeedMPS,
        gamePieceRotation);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTouchGroundHeight(0.2);
  }

  public enum CarrotStations {
    BLUE_RAMP(
        new Pose2d(FieldConstants.BLUE_RAMP.getTranslation(), Rotation2d.kCW_90deg),
        Centimeters.of(57.659358),
        new Translation2d(1.816, 0)),
    RED_RAMP(
        new Pose2d(FieldConstants.RED_RAMP.getTranslation(), Rotation2d.kCCW_90deg),
        Centimeters.of(57.659358),
        new Translation2d(-1.816, 0)),
    BLUE_REAR_DEPOT(
        new Pose2d(FieldConstants.BLUE_REAR_DEPOT.getTranslation(), Rotation2d.kCW_90deg),
        Centimeters.of(58.420),
        new Translation2d(-1.816, 0)),
    BLUE_SIDE_DEPOT(
        new Pose2d(FieldConstants.BLUE_SIDE_DEPOT.getTranslation(), Rotation2d.kZero),
        Centimeters.of(58.420),
        new Translation2d(0, 1.816)),
    RED_REAR_DEPOT(
        new Pose2d(FieldConstants.RED_REAR_DEPOT.getTranslation(), Rotation2d.kCW_90deg),
        Centimeters.of(58.420),
        new Translation2d(1.816, 0)),
    RED_SIDE_DEPOT(
        new Pose2d(FieldConstants.RED_SIDE_DEPOT.getTranslation(), Rotation2d.kZero),
        Centimeters.of(58.420),
        new Translation2d(0, 1.816));

    private final Pose2d startingPose;
    private final Distance height;
    private final Translation2d droppingVelocity;

    CarrotStations(Pose2d startingPose, Distance height, Translation2d droppingVelocity) {
      this.startingPose = startingPose;
      this.height = height;
      this.droppingVelocity = droppingVelocity;
    }
  }

  public static HarvestHavocCarrotOnFly dropFromCarrotStation(CarrotStations station) {
    return new HarvestHavocCarrotOnFly(
        station.startingPose.getTranslation(),
        station.droppingVelocity,
        station.height.in(Meters),
        -0.4,
        new Rotation3d(station.startingPose.getRotation()));
  }

  @Override
  public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
    if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
    simulatedArena.addGamePiece(
        new GamePieceOnFieldSimulation(
            HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO,
            () ->
                Math.max(
                    HarvestHavocCarrotOnField.HARVEST_HAVOC_CARROT_INFO.gamePieceHeight().in(Meters)
                        / 2,
                    getPositionAtTime(super.launchedTimer.get()).getZ()),
            new Pose2d(
                getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                gamePieceRotation.toRotation2d()),
            super.initialLaunchingVelocityMPS));
  }
}
