package frc.robot.util.io.motors;

public interface MotorIO {
  abstract class MotorIOInputs {
    public boolean connected;
    public double appliedVoltage;
    public double statorCurrentAmps;
    public double tempCelsius;

    public boolean[] followerConnected = new boolean[0];
    public double[] followerTempCelsius = new double[0];
  }

  enum MotorIOMode {
    COAST,
    BRAKE,
    VOLTAGE_CONTROL,
    POSITION_CONTROL,
    VELOCITY_CONTROL,
  }

  record RotationalMechanismConstraints(
      double reduction,
      double moi,
      double radiusMeters,
      double minAngleRads,
      double maxAngleRads,
      double startingAngleRads) {}

  record LinearMechanismConstraints(
      double reduction,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters) {}

  default void configure(boolean positionControl, boolean velocityControl) {}

  default void setVoltage(double volts) {}

  /**
   * Send a custom output request to the motor through a user-defined callback. Currently only
   * supported on TalonFX.
   */
  default void setOutput(int slot, double output) {}

  /**
   * Run a user-defined callback. These can be configured in the MotorIO constructors. Currently
   * only supported on TalonFX.
   */
  default void runCallback(int slot) {}

  default void coast() {}

  default void brake() {}

  default int getNumFollowers() {
    return 0;
  }
}
