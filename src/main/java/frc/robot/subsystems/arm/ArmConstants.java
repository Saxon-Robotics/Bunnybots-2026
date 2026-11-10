package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public final class ArmConstants {
  public static final boolean ENABLED = false;

  public static final double SETPOINT_TOLERANCE = 2.5; // deg
  public static final double STOW_DELAY = 1.0;

  // physical constraints
  public static final double GEAR_RATIO = 64;
  public static final double RADIUS = Units.inchesToMeters(18.6); // m
  public static final double MASS = Units.lbsToKilograms(15.0); // kg
  public static final double MIN_ANGLE_RAD = 0.0;
  public static final double MAX_ANGLE_RAD = Math.toRadians(100);

  public static final double MAX_PIVOT_VELOCITY = 10;
  public static final double MAX_PIVOT_ACCELERATION = 10;

  public enum Setpoint {
    NONE(Rotations.of(0)),
    STOWED(Rotations.of(0)),
    // intake
    RAMP(Degrees.of(0)),
    GROUND(Degrees.of(90)),
    // outtake
    OVEN(Degrees.of(30)),
    PANTRY(Degrees.of(45)),
    // tung
    TONGUE(Degrees.of(30));

    public final Angle angle;

    Setpoint(Angle angle) {
      this.angle = angle;
    }
  }

  public static final TalonFXConfiguration PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                  .withFeedbackRemoteSensorID(Constants.CANConstants.ARM_ENCODER)
                  .withRotorToSensorRatio(GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(142.248)
                  .withKI(0)
                  .withKD(2.487)
                  .withKS(0)
                  .withKV(7.540)
                  .withKA(0.096)
                  .withKG(0.316)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(MAX_PIVOT_VELOCITY)
                  .withMotionMagicAcceleration(MAX_PIVOT_ACCELERATION));

  public static final CANcoderConfiguration ENCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(Degrees.of(0)));
}
