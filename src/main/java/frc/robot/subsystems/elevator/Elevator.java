package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.elevator.ElevatorConstants.SETPOINTS;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  public enum Setpoint {
    NONE(false),
    STOWED(false),
    RAMP(true),
    L1(true),
    L2(true),
    L3(true);

    public final boolean isForScoring;

    Setpoint(boolean forScoring) {
      this.isForScoring = forScoring;
    }
  }

  private final LinearSystem elevator;

  @Getter private Setpoint setpoint = Setpoint.NONE;
  private double setpointRad;
  private boolean coastOverride;

  public Elevator() {
    LinearSystemIO io =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX.Builder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.ELEVATOR_LEFT,
                  ElevatorConstants.MOTOR_CONFIG)
              .addFollower(Constants.CANConstants.ELEVATOR_RIGHT, MotorAlignmentValue.Aligned)
              .build();
          case SIM -> new LinearSystemIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.LinearMechanismConstraints(
                  ElevatorConstants.GEAR_RATIO,
                  ElevatorConstants.CARRIAGE_MASS,
                  ElevatorConstants.DRUM_RADIUS,
                  ElevatorConstants.MIN_HEIGHT_METERS,
                  ElevatorConstants.MAX_HEIGHT_METERS),
              ElevatorConstants.ELEVATOR_KP,
              ElevatorConstants.ELEVATOR_KD,
              0);
          case REPLAY -> new LinearSystemIO() {};
        };
    elevator =
        new LinearSystem.Builder("Elevator", io)
            .setDrumRadius(ElevatorConstants.DRUM_RADIUS)
            .setBrakeMode(() -> !coastOverride)
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
    Angle newSetpoint = SETPOINTS.get(setpoint);
    elevator.runPosition(newSetpoint);
    this.setpoint = setpoint;
    this.setpointRad = newSetpoint.in(Radians);
    Logger.recordOutput("Elevator/State", this.setpoint.toString());
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
              double magnitude = joystick.getAsDouble();
              elevator.runVoltage(
                  Math.copySign(magnitude * magnitude, magnitude)
                      * ElevatorConstants.MAX_MANUAL_VOLTAGE);
            })
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public Command homingSequence() {
    Debouncer homingDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    Timer homingTimer = new Timer();

    return startRun(
            () -> {
              runSetpoint(Setpoint.NONE);
              Logger.recordOutput("Elevator/State", "HOMING");
              elevator.runVoltage(-ElevatorConstants.HOMING_VOLTAGE);
            },
            () -> {
              if (homingDebouncer.calculate(
                      elevator.getVelocityRadPerSec()
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

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput(
        "Elevator/RobotComponents",
        // elevator
        new Pose3d(0.0, 0.0, getPositionMeters(), Rotation3d.kZero));
  }
}
