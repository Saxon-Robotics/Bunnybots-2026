package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class ElevatorConstants {
  public static Angle distanceToAngle(Distance distance) {
    return Rotations.of(Units.radiansToRotations(distance.in(Meters) / DRUM_RADIUS) / 2);
  }

  public static Distance angleToDistance(Angle angle) {
    return Meters.of(2 * angle.in(Radians) * DRUM_RADIUS);
  }

  public static final double SETPOINT_TOLERANCE = 1.0; // rad

  public static final double MAX_MANUAL_VOLTAGE = 2.5;
  public static final double HOMING_VOLTAGE = 1.0;
  public static final double HOMING_VELOCITY_THRESHOLD = 0.1; // placeholder, find this

  // physical constraints
  public static final double GEAR_RATIO = 9;
  public static final double CARRIAGE_MASS = Units.lbsToKilograms(13.0);
  public static final double DRUM_RADIUS = Units.inchesToMeters(0.8785);
  public static final double MIN_HEIGHT_METERS = 0;
  public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(45);

  public static final double MAX_VELOCITY = 90.008;
  public static final double MAX_ACCELERATION = 5440.138;

  public enum Setpoint {
    NONE(-1, Meters.of(0)),
    STOWED(-1, Meters.of(0)),
    RAMP(-1, Inches.of(9)),
    L1(0, Inches.of(21)),
    L2(1, Inches.of(33)),
    L3(2, Inches.of(48));

    public final int scoringIndex;
    public final Angle angle;

    Setpoint(int scoringIndex, Distance height) {
      this.scoringIndex = scoringIndex;
      this.angle = distanceToAngle(height);
    }
  }

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  // recalc values
                  .withKP(2.9)
                  .withKI(0)
                  .withKD(0.002)
                  .withKS(0)
                  .withKV(0.118)
                  .withKA(0.000)
                  .withKG(0.35)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                  .withMotionMagicAcceleration(MAX_ACCELERATION));
}
