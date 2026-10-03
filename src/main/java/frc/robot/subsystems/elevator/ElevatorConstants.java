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
import java.util.EnumMap;
import java.util.Map;

public final class ElevatorConstants {
  public static Angle distanceToAngle(Distance meters) {
    return Rotations.of(Units.radiansToRotations(meters.in(Meters) / DRUM_RADIUS));
  }

  public static final double SETPOINT_TOLERANCE = 0.06;

  public static final double MAX_MANUAL_VOLTAGE = 6.0;
  public static final double HOMING_VOLTAGE = 2.0;
  public static final double HOMING_VELOCITY_THRESHOLD = 0.1; // placeholder, find this

  // physical constraints
  public static final double GEAR_RATIO = 4;
  public static final double CARRIAGE_MASS = Units.lbsToKilograms(13.0);
  public static final double DRUM_RADIUS = Units.inchesToMeters(0.8785);
  public static final double MIN_HEIGHT_METERS = 0;
  public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(34.5);

  public static final Map<Elevator.Setpoint, Angle> SETPOINTS =
      new EnumMap<>(Elevator.Setpoint.class);

  static {
    SETPOINTS.put(Elevator.Setpoint.STOWED, Rotations.of(0));
    SETPOINTS.put(Elevator.Setpoint.RAMP, distanceToAngle(Meters.of(0)));
    SETPOINTS.put(Elevator.Setpoint.L1, distanceToAngle(Inches.of(20)));
    SETPOINTS.put(Elevator.Setpoint.L2, distanceToAngle(Inches.of(30)));
    SETPOINTS.put(Elevator.Setpoint.L3, distanceToAngle(Meters.of(0)));
  }

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  // recalc values
                  .withKP(2.071)
                  .withKI(0)
                  .withKD(0.026)
                  .withKS(0)
                  .withKV(0.118)
                  .withKA(0.001)
                  .withKG(0.311)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(89.311)
                  .withMotionMagicAcceleration(1404.071));
}
