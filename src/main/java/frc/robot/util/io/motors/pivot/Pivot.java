package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.sensors.encoder.EncoderIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends Motor<PivotIO, PivotIOInputsAutoLogged> {
  private Translation3d pivotOffset;
  private Rotation2d pivotRotation;
  private double intialRotation;

  public Pivot(String name, PivotIO io, EncoderIO encoderIO, BooleanSupplier brakeMode) {
    super(name, io, new PivotIOInputsAutoLogged(), encoderIO, brakeMode);
    io.configure(true, false);
  }

  public Pivot(String name, PivotIO io, EncoderIO encoderIO) {
    this(name, io, encoderIO, RobotStateHandler::isRobotEnabled);
  }

  public Pivot(String name, PivotIO io) {
    this(name, io, in -> {});
  }

  /**
   * Set up visualization for this mechanism.
   *
   * @param offset Pivot offset. If this is set to null, visualization will be disabled.
   * @return The modified {@link LinearSystem.Builder} object for method chaining.
   */
  public Pivot enableVisualization(Translation3d offset, double rotation, double initial) {
    this.pivotOffset = offset;
    this.pivotRotation = new Rotation2d(rotation);
    this.intialRotation = initial;
    return this;
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();

    if (pivotOffset != null) {
      Logger.recordOutput(
          name + "/RobotComponents",
          new Pose3d(
              pivotOffset,
              new Rotation3d(
                  0.0, inputs.positionDeg + intialRotation, pivotRotation.getRadians())));
    }
  }

  public void runPosition(int slot, Angle angle) {
    if (tempCritical) return;

    io.setPosition(slot, angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", angle.in(Degrees));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runPosition(Angle angle) {
    runPosition(0, angle);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(inputs.velocityDegPerSec);
  }

  public double getVelocityDegPerSec() {
    return inputs.velocityDegPerSec;
  }
}
