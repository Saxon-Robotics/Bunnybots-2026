package frc.robot.util.io.motors.elevator;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.io.motors.MotorIOSim;

public class LinearSystemIOSim extends MotorIOSim implements LinearSystemIO {
  private final ElevatorSim sim;
  private ProfiledPIDController profiledPID;
  private ElevatorFeedforward feedforward;

  private final double gearing;
  private final double drumRadiusMeters;

  private double targetPositionRad = 0.0;

  public LinearSystemIOSim(
      DCMotor gearbox,
      LinearMechanismConstraints constraints,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);

    this.gearing = constraints.reduction();
    this.drumRadiusMeters = constraints.drumRadiusMeters();

    this.sim =
        new ElevatorSim(
            gearbox,
            gearing,
            constraints.carriageMassKg(),
            drumRadiusMeters,
            constraints.minHeightMeters(),
            constraints.maxHeightMeters(),
            true,
            0.0);
  }

  /**
   * Set up the simulation to use profiled trapezoidal PID instead of standard PID
   *
   * @param maxVelocity Max velocity of the elevator, in radians per second.
   * @param maxAcceleration Max acceleration of the elevator, in radians per second squared.
   * @return The modified {@link LinearSystemIOSim} object for method chaining.
   */
  public LinearSystemIOSim withProfiledPID(double maxVelocity, double maxAcceleration) {
    profiledPID =
        new ProfiledPIDController(
            pid.getP(),
            pid.getI(),
            pid.getD(),
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    return this;
  }

  public LinearSystemIOSim withFeedforward(double kS, double kG, double kV, double kA) {
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    return this;
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    if (isClosedLoop) {
      double currentMotorRad = carriageMetersToRad(sim.getPositionMeters());
      if (profiledPID != null) {
        appliedVoltage =
            MathUtil.clamp(
                profiledPID.calculate(currentMotorRad, targetPositionRad)
                    + (feedforward != null
                        ? feedforward.calculate(profiledPID.getSetpoint().velocity)
                        : 0),
                -12,
                12);
      } else {
        appliedVoltage = MathUtil.clamp(pid.calculate(currentMotorRad, targetPositionRad), -12, 12);
      }
    }
    updateMotorInputs(inputs);

    sim.setInputVoltage(appliedVoltage);
    sim.update(0.020);
    inputs.positionRad = carriageMetersToRad(sim.getPositionMeters());
    inputs.velocityRadPerSec = carriageMetersToRad(sim.getVelocityMetersPerSecond());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(Angle angle) {
    this.targetPositionRad = angle.in(Radians);
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(Angle angle) {
    sim.setState(radsToCarriageMeters(angle.in(Radians)), 0.0);
  }

  private double carriageMetersToRad(double meters) {
    double drumRotations = meters / drumRadiusMeters;
    return drumRotations * gearing;
  }

  private double radsToCarriageMeters(double radians) {
    double drumRadians = radians / gearing;
    return drumRadians * drumRadiusMeters;
  }
}
