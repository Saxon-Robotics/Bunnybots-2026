package frc.robot.util.io.sensors.lasercan;

import org.littletonrobotics.junction.AutoLog;

@FunctionalInterface
public interface LaserCanIO {
  @AutoLog
  class LaserCanIOInputs {
    public boolean connected;
    public boolean measurementValid;
    public double distanceMillimeters;
  }

  void updateInputs(LaserCanIOInputs inputs);
}
