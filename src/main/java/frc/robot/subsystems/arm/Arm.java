package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants.Setpoint;
import frc.robot.util.LoggedTracer;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.pivot.Pivot;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.pivot.PivotIOSim;
import frc.robot.util.io.sensors.encoder.EncoderIO;
import frc.robot.util.io.sensors.encoder.EncoderIOCANcoder;
import frc.robot.util.subsystems.ExtendedSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends ExtendedSubsystem {
  private final Pivot pivot;

  private final Alert disconnectedAlert =
      new Alert("Pivot motor disconnected.", Alert.AlertType.kError);
  private final Alert encoderDisconnectedAlert =
      new Alert("Pivot encoder disconnected.", Alert.AlertType.kError);

  @Getter private Setpoint setpoint = Setpoint.NONE;
  private double setpointDeg;
  private boolean coastOverride;

  private final Timer stowDelay = new Timer();

  public Arm() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> ArmConstants.ENABLED
              ? new MotorIOTalonFX.Builder(
                      Constants.CANConstants.SUPERSTRUCTURE,
                      Constants.CANConstants.ARM,
                      ArmConstants.PIVOT_CONFIG)
                  .addControlRequest(new MotionMagicVoltage(0))
                  .build()
              : new PivotIO() {};
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  ArmConstants.GEAR_RATIO,
                  SingleJointedArmSim.estimateMOI(ArmConstants.RADIUS, ArmConstants.MASS),
                  ArmConstants.RADIUS,
                  // placeholders, todo figure them out
                  0,
                  0,
                  0),
              ArmConstants.PIVOT_CONFIG.Slot0.kP,
              ArmConstants.PIVOT_CONFIG.Slot0.kD,
              0);
          default -> new PivotIO() {};
        };
    EncoderIO encoderIO =
        switch (Constants.currentMode) {
          case REAL -> ArmConstants.ENABLED
              ? new EncoderIOCANcoder(
                  Constants.CANConstants.SUPERSTRUCTURE,
                  Constants.CANConstants.ARM_ENCODER,
                  ArmConstants.ENCODER_CONFIG)
              : inputs -> inputs.absolutePosition = setpoint.angle;
          default -> inputs -> {};
        };

    pivot = new Pivot("Arm", pivotIO, encoderIO, () -> !coastOverride);

    if (!ArmConstants.ENABLED) {
      new Alert(
              "End effector pivot is currently disabled. To enable it, set ArmConstants.ENABLED to true.",
              Alert.AlertType.kWarning)
          .set(true);
    }

    Logger.recordOutput("Arm/State", setpoint.toString());
  }

  @Override
  public void periodic() {
    pivot.periodic();
    if (stowDelay.hasElapsed(ArmConstants.STOW_DELAY)) {
      stow();
    }
    if (ArmConstants.ENABLED) {
      disconnectedAlert.set(!pivot.isConnected());
      encoderDisconnectedAlert.set(!pivot.isEncoderConnected());
    }
    LoggedTracer.record("Arm/Periodic");
  }

  private void runSetpoint(Setpoint setpoint) {
    stowDelay.stop();
    stowDelay.reset();
    if (setpoint == Setpoint.NONE) {
      this.setpoint = setpoint;
      return;
    }
    Angle newSetpoint = setpoint.angle;
    pivot.runPosition(newSetpoint);
    this.setpoint = setpoint;
    this.setpointDeg = newSetpoint.in(Degrees);
    Logger.recordOutput("Arm/State", this.setpoint.toString());
    Logger.recordOutput("Arm/GoalAngle", newSetpoint);
  }

  private void stow() {
    runSetpoint(Setpoint.STOWED);
  }

  public Command ramp() {
    return startEnd(() -> runSetpoint(Setpoint.RAMP), this::stow);
  }

  public Command ground() {
    return startEnd(() -> runSetpoint(Setpoint.GROUND), stowDelay::start);
  }

  public Command oven() {
    return startEnd(() -> runSetpoint(Setpoint.OVEN), this::stow);
  }

  public Command pantry() {
    return startEnd(() -> runSetpoint(Setpoint.PANTRY), this::stow);
  }

  public Command tongue() {
    return startEnd(() -> runSetpoint(Setpoint.TONGUE), this::stow);
  }

  public double getPositionDeg() {
    return pivot.getAbsolutePosition().in(Degrees);
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean hasReachedSetpoint() {
    if (setpoint == Setpoint.NONE) return false;
    return Math.abs(setpointDeg - getPositionDeg()) < ArmConstants.SETPOINT_TOLERANCE;
  }
}
