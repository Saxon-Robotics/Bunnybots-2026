package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.sensors.encoder.EncoderIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends Motor<PivotIO, PivotIOInputsAutoLogged> {
  private Transform3d pivotOffset;

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
   * @param offset Pivot offset. The pivot will move rotationally in the pitch (Y) axis. This can be
   *     changed by adding a yaw or roll transform. If this is set to null, visualization will be
   *     disabled.
   * @return The modified {@link Pivot} object for method chaining.
   */
  public Pivot enableVisualization(Transform3d offset) {
    this.pivotOffset = offset;
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
              pivotOffset.getTranslation(),
              pivotOffset
                  .getRotation()
                  .plus(new Rotation3d(0, Math.toRadians(getPositionDeg()), 0))));
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
