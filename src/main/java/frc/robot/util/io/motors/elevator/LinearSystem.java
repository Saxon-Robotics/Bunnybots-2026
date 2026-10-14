package frc.robot.util.io.motors.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.sensors.encoder.EncoderIO;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class LinearSystem extends Motor<LinearSystemIO, LinearSystemIOInputsAutoLogged> {
  private final Function<Distance, Angle> distanceToAngle;
  private final Function<Angle, Distance> angleToDistance;
  private final Pose3d[] visualizedComponents;

  private LinearSystem(
      String name,
      LinearSystemIO io,
      EncoderIO encoderIO,
      BooleanSupplier brakeMode,
      Function<Distance, Angle> distanceToAngle,
      Function<Angle, Distance> angleToDistance,
      int numStages) {
    super(name, io, new LinearSystemIOInputsAutoLogged(), encoderIO, brakeMode);
    io.configure(true, true);
    this.distanceToAngle = distanceToAngle;
    this.angleToDistance = angleToDistance;
    this.visualizedComponents = new Pose3d[numStages];
  }

  /** {@inheritDoc} */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();

    if (visualizedComponents.length > 0) {
      double carriagePosition = angleToDistance.apply(getPosition()).in(Meters);
      for (int i = 0; i < visualizedComponents.length - 1; i++) {
        double stageHeight = carriagePosition * (i + 1) / visualizedComponents.length;
        visualizedComponents[i] = new Pose3d(0, 0, stageHeight, Rotation3d.kZero);
      }
      visualizedComponents[visualizedComponents.length - 1] =
          new Pose3d(0, 0, carriagePosition, Rotation3d.kZero);
      Logger.recordOutput(name + "/RobotComponents", visualizedComponents);
    }
  }

  public void runPosition(Angle angle) {
    if (tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointRad", angle.in(Radians));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runPosition(Distance position) {
    if (distanceToAngle == null) {
      throw new IllegalStateException(
          "Drum radius or custom position function must be set in the builder in order to use linear position requests!");
    }
    runPosition(distanceToAngle.apply(position));
  }

  public void runVelocity(double rps) {
    if (tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Radians.of(inputs.positionRad);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(inputs.velocityRadPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public static class Builder {
    private final String name;
    private final LinearSystemIO io;
    private EncoderIO encoderIO = inputs -> {};
    private BooleanSupplier brakeMode = () -> true;
    private Function<Distance, Angle> distanceToAngle;
    private Function<Angle, Distance> angleToDistance;
    private int numStages = 0;

    public Builder(String name, LinearSystemIO io) {
      this.name = name;
      this.io = io;
    }

    public Builder addEncoder(EncoderIO encoderIO) {
      this.encoderIO = encoderIO;
      return this;
    }

    public Builder setBrakeMode(BooleanSupplier brakeMode) {
      this.brakeMode = brakeMode;
      return this;
    }

    public Builder setDrumRadius(double drumRadiusMeters) {
      // don't override if manually set
      if (distanceToAngle == null) {
        distanceToAngle = distance -> Radians.of(distance.in(Meters) / drumRadiusMeters);
      }
      if (angleToDistance == null) {
        angleToDistance = angle -> Meters.of(angle.in(Radians) * drumRadiusMeters);
      }
      return this;
    }

    public Builder useCustomPositionFunctions(
        Function<Distance, Angle> distanceToAngle, Function<Angle, Distance> angleToDistance) {
      this.distanceToAngle = distanceToAngle;
      this.angleToDistance = angleToDistance;
      return this;
    }

    /**
     * Set up visualization for this mechanism.
     *
     * @param numStages Number of stages in the elevator. If this is set to 0, visualization will be
     *     disabled.
     * @return The modified {@link Builder} object for method chaining.
     */
    public Builder enableVisualization(int numStages) {
      this.numStages = numStages;
      return this;
    }

    public LinearSystem build() {
      // we don't trust the user lol
      if (numStages <= 0) {
        numStages = 0;
      } else if (angleToDistance == null) {
        throw new IllegalStateException(
            "Drum radius or custom position functions must be set in the builder in order to visualize the mechanism!");
      }

      return new LinearSystem(
          name, io, encoderIO, brakeMode, distanceToAngle, angleToDistance, numStages);
    }
  }
}
