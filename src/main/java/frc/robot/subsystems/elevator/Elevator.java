package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.elevator.ElevatorConstants.MOTOR_CONFIG;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.Setpoint;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.elevator.LinearSystem;
import frc.robot.util.io.motors.elevator.LinearSystemIO;
import frc.robot.util.io.motors.elevator.LinearSystemIOSim;
import frc.robot.util.subsystems.ExtendedSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends ExtendedSubsystem {
  private final LinearSystem elevator;

  @Getter private ElevatorConstants.Setpoint setpoint = Setpoint.NONE;
  private double setpointRad;
  private boolean coastOverride;

  public Elevator() {
    LinearSystemIO io =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.ELEVATOR_LEFT,
                  ElevatorConstants.MOTOR_CONFIG)
              .addControlRequest(new MotionMagicVoltage(0))
              .addFollower(Constants.CANConstants.ELEVATOR_RIGHT, MotorAlignmentValue.Opposed)
              .setSignalUpdateFrequency(500.0, 50.0)
              .build();
          case SIM -> new LinearSystemIOSim(
                  DCMotor.getKrakenX60(1),
                  new MotorIO.LinearMechanismConstraints(
                      ElevatorConstants.GEAR_RATIO,
                      ElevatorConstants.CARRIAGE_MASS,
                      ElevatorConstants.DRUM_RADIUS,
                      ElevatorConstants.MIN_HEIGHT_METERS,
                      ElevatorConstants.MAX_HEIGHT_METERS),
                  0.3296,
                  0.0041,
                  1)
              .withProfiledPID(
                  Units.rotationsToRadians(ElevatorConstants.MAX_VELOCITY),
                  Units.rotationsToRadians(ElevatorConstants.MAX_ACCELERATION))
              .withFeedforward(0, MOTOR_CONFIG.Slot0.kG, 0.005, 0.00016);
          case REPLAY -> new LinearSystemIO() {};
        };
    elevator =
        new LinearSystem.Builder("Elevator", io)
            .setDrumRadius(ElevatorConstants.DRUM_RADIUS)
            .setBrakeMode(() -> !coastOverride)
            .enableVisualization(2)
            .build();

    Logger.recordOutput("Elevator/State", this.setpoint.toString());
  }

  @Override
  public void disable() {
    elevator.stop();
  }

  @Override
  public void periodic() {
    elevator.periodic();
  }

  private void runSetpoint(Setpoint setpoint) {
    if (setpoint == Setpoint.NONE) {
      this.setpoint = setpoint;
      return;
    }
    Angle newSetpoint = setpoint.angle;
    elevator.runPosition(newSetpoint);
    this.setpoint = setpoint;
    this.setpointRad = newSetpoint.in(Radians);
    Logger.recordOutput("Elevator/State", this.setpoint.toString());
    Logger.recordOutput("Elevator/GoalHeight", ElevatorConstants.angleToDistance(newSetpoint));
  }

  public Command stow() {
    return startEnd(() -> runSetpoint(Setpoint.STOWED), () -> {}).until(this::hasReachedSetpoint);
  }

  public Command ramp() {
    return startEnd(() -> runSetpoint(Setpoint.RAMP), () -> {}).until(this::hasReachedSetpoint);
  }

  public Command l1() {
    return startEnd(() -> runSetpoint(Setpoint.L1), () -> {}).until(this::hasReachedSetpoint);
  }

  public Command l2() {
    return startEnd(() -> runSetpoint(Setpoint.L2), () -> {}).until(this::hasReachedSetpoint);
  }

  public Command l3() {
    return startEnd(() -> runSetpoint(Setpoint.L3), () -> {}).until(this::hasReachedSetpoint);
  }

  public Command manualControl(DoubleSupplier joystick) {
    return startRun(
            () -> {
              runSetpoint(Setpoint.NONE);
              Logger.recordOutput("Elevator/State", "MANUAL");
            },
            () -> {
              double magnitude = MathUtil.clamp(joystick.getAsDouble(), -1.0, 1.0);
              elevator.runVoltage(
                  Math.copySign(magnitude * magnitude, magnitude)
                      * ElevatorConstants.MAX_MANUAL_VOLTAGE);
            })
        .finallyDo(elevator::stop)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public Command homingSequence() {
    Debouncer homingDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    Timer homingTimer = new Timer();

    return startRun(
            () -> {
              runSetpoint(Setpoint.NONE);
              Logger.recordOutput("Elevator/State", "HOMING");
              homingDebouncer.calculate(false);
              elevator.runVoltage(-ElevatorConstants.HOMING_VOLTAGE);
            },
            () -> {
              if (homingDebouncer.calculate(
                      Math.abs(elevator.getVelocityRadPerSec())
                          <= ElevatorConstants.HOMING_VELOCITY_THRESHOLD)
                  && !homingTimer.isRunning()) {
                elevator.stop();
                elevator.resetPosition(Rotations.of(0));
                // wait for operation to finish
                homingTimer.start();
              }
            })
        .until(() -> homingTimer.hasElapsed(0.101))
        .finallyDo(
            () -> {
              homingTimer.stop();
              homingTimer.reset();
              runSetpoint(Setpoint.STOWED);
            });
  }

  public Command release() {
    return startEnd(
        () -> {
          coastOverride = true;
          elevator.stop();
        },
        () -> coastOverride = false);
  }

  public double getPositionRad() {
    return elevator.getPositionRad();
  }

  @AutoLogOutput(key = "Elevator/HeightMeters")
  public double getPositionMeters() {
    return elevator.getPositionRad() * ElevatorConstants.DRUM_RADIUS;
  }

  public boolean hasReachedSetpoint() {
    return Math.abs(setpointRad - getPositionRad()) < ElevatorConstants.SETPOINT_TOLERANCE;
  }
}
