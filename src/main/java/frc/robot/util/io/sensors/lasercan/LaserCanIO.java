package frc.robot.util.io.sensors.lasercan;

import java.util.function.BooleanSupplier;
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

  static LaserCanIO beambreakSim(BooleanSupplier beambreak, double threshold) {
    return inputs -> {
      inputs.connected = true;
      inputs.measurementValid = true;
      inputs.distanceMillimeters = beambreak.getAsBoolean() ? threshold - 1 : threshold + 1;
    };
  }
}
