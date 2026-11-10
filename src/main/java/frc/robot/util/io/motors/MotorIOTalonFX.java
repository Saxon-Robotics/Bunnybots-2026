package frc.robot.util.io.motors;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.io.motors.elevator.LinearSystemIO;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.sensors.encoder.EncoderIOCANcoder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class MotorIOTalonFX implements AutoCloseable, RollerIO, PivotIO, LinearSystemIO {
  private final TalonFX leader;
  private final TalonFX[] followers;

  private VelocityVoltage velocityRequest;
  private PositionRequest[] positionRequests;
  private final ControlRequest[] customRequests;
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake brakeRequest = new StaticBrake();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;
  private final BaseStatusSignal[] followerTemps;

  private final AtomicReference<Angle> angleResetVal = new AtomicReference<>();
  private Notifier resetPosition;

  @FunctionalInterface
  private interface PositionRequest {
    void apply(TalonFX motor, Angle angle);
  }

  @FunctionalInterface
  public interface ControlRequest {
    void apply(TalonFX motor, double output);
  }

  @SuppressWarnings("resource")
  private MotorIOTalonFX(
      CANBus canbus,
      int id,
      TalonFXConfiguration config,
      int[] followerIds,
      MotorAlignmentValue[] followerAlignments,
      PositionRequest[] positionRequests,
      ControlRequest[] customRequests,
      EncoderIOCANcoder encoder,
      double primarySignalUpdateFreq,
      double secondarySignalUpdateFreq,
      double optimizedSignalUpdateFreq) {
    // Instantiate motors
    leader = new TalonFX(id, canbus);
    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new TalonFX(followerIds[i], canbus);
    }
    // Configure motors
    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    for (TalonFX follower : followers) {
      tryUntilOk(5, () -> follower.getConfigurator().apply(config));
    }
    // Create status signals
    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    temp = leader.getDeviceTemp();
    followerTemps = new BaseStatusSignal[followers.length];
    for (int i = 0; i < followerTemps.length; i++) {
      followerTemps[i] = followers[i].getDeviceTemp();
    }
    // Register status signals
    BaseStatusSignal.setUpdateFrequencyForAll(primarySignalUpdateFreq, position, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        secondarySignalUpdateFreq, voltage, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(secondarySignalUpdateFreq, followerTemps);
    leader.optimizeBusUtilization(optimizedSignalUpdateFreq);
    ParentDevice.optimizeBusUtilizationForAll(optimizedSignalUpdateFreq, followers);
    PhoenixUtil.registerSignals(canbus, position, velocity, voltage, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    tryUntilOk(5, () -> leader.setPosition(0));
    // Set follower behavior
    if (followers.length != followerAlignments.length) {
      throw new IllegalArgumentException("Every follower motor must have an alignment value!");
    }
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
    this.positionRequests = positionRequests;
    this.customRequests = customRequests;
    // Configure feedback
    if (encoder != null) {
      tryUntilOk(
          5,
          () ->
              leader
                  .getConfigurator()
                  .apply(
                      config.Feedback.withFeedbackSensorSource(
                              FeedbackSensorSourceValue.FusedCANcoder)
                          .withFeedbackRemoteSensorID(encoder.getDeviceID())));
    }
  }

  /**
   * Constructs a new MotorIOTalonFX.
   *
   * @param canbus The CAN bus this device is connected on
   * @param id The CAN ID of the leader motor
   * @param config The config to apply to the motors
   * @param followerIds The CAN IDs of the follower motors
   * @param followerAlignments The alignments of the follower motors relative to the leader
   * @deprecated Use {@link MotorIOTalonFX.Builder} instead, which provides a more streamlined setup
   *     process with more customizability.
   */
  @Deprecated(since = "2.1.0")
  public MotorIOTalonFX(
      CANBus canbus,
      int id,
      TalonFXConfiguration config,
      int[] followerIds,
      MotorAlignmentValue[] followerAlignments) {
    this(
        canbus,
        id,
        config,
        followerIds,
        followerAlignments,
        new PositionRequest[0],
        new ControlRequest[0],
        null,
        100.0,
        50.0,
        4.0);
  }

  /**
   * Constructs a new MotorIOTalonFX.
   *
   * @param canbus The CAN bus this device is connected on
   * @param id The CAN ID of the leader motor
   * @param config The config to apply to the motors
   * @deprecated Use {@link MotorIOTalonFX.Builder} instead, which provides a more streamlined setup
   *     process with more customizability.
   */
  @Deprecated(since = "2.1.0")
  public MotorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, config, new int[0], new MotorAlignmentValue[0]);
  }

  @Override
  public void configure(boolean positionControl, boolean velocityControl) {
    if (positionControl) {
      configurePositionControl();
    }
    if (velocityControl) {
      configureVelocityControl();
    }
  }

  private void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(voltage, statorCurrent, temp);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    for (int i = 0; i < followerTemps.length; i++) {
      inputs.followerConnected[i] = followerTemps[i].getStatus().isOK();
      inputs.followerTempCelsius[i] = followerTemps[i].getValueAsDouble();
    }
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRPS = velocity.getValueAsDouble();
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionDeg = position.getValue().in(Degrees);
    inputs.velocityDegPerSec = velocity.getValue().in(DegreesPerSecond);
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(LinearSystemIOInputs inputs) {
    inputs.positionRad = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    updateMotorInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle angle) {
    positionRequests[0].apply(leader, angle);
  }

  @Override
  public void setPosition(int slot, Angle angle) {
    if (slot < 0 || slot >= positionRequests.length) return;
    positionRequests[slot].apply(leader, angle);
  }

  @Override
  public void setVelocity(double rps) {
    leader.setControl(velocityRequest.withVelocity(rps));
  }

  /**
   * This method is designed to allow using other compatible ControlRequest types from the Phoenix 6
   * API with this motor that are not explicitly offered in the {@link Builder} methods. This method
   * may also be used to call user-defined callbacks that require the raw motor object.
   */
  @Override
  public void setOutput(int slot, double output) {
    if (slot < 0 || slot >= customRequests.length) return;
    customRequests[slot].apply(leader, output);
  }

  /** This method may be used to call user-defined callbacks that require the raw motor object. */
  @Override
  public void runCallback(int slot) {
    if (slot < 0 || slot >= customRequests.length) return;
    customRequests[slot].apply(leader, 0);
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }

  @Override
  public void brake() {
    leader.setControl(brakeRequest);
  }

  @Override
  public void resetPosition(Angle angle) {
    angleResetVal.set(angle);
    resetPosition.startSingle(0);
  }

  @Override
  public int getNumFollowers() {
    return followers.length;
  }

  @Override
  public void close() {
    if (resetPosition != null) {
      resetPosition.stop();
      resetPosition.close();
    }
    leader.close();
    for (TalonFX follower : followers) {
      follower.close();
    }
  }

  /**
   * Returns the unwrapped motor object for more advanced control. Not recommended for regular use;
   * may cause unexpected behavior.
   */
  public TalonFX getRawIO() {
    return leader;
  }

  /**
   * Returns the unwrapped follower motor object(s) for more advanced control. Not recommended for
   * regular use; may cause unexpected behavior.
   */
  public TalonFX[] getRawFollowerIO() {
    return followers;
  }

  private void configurePositionControl() {
    if (resetPosition != null) return;
    if (positionRequests == null || positionRequests.length == 0) {
      PositionVoltage defaultRequest = new PositionVoltage(0);
      positionRequests =
          new PositionRequest[] {
            (leader, angle) -> leader.setControl(defaultRequest.withPosition(angle))
          };
    }
    resetPosition = new Notifier(() -> leader.setPosition(angleResetVal.get()));
  }

  private void configureVelocityControl() {
    if (velocityRequest != null) return;
    velocityRequest = new VelocityVoltage(0);
  }

  public static class Builder {
    private final CANBus canbus;
    private final int id;
    private final TalonFXConfiguration config;
    private final List<Pair<Integer, MotorAlignmentValue>> followers = new ArrayList<>();
    private final List<PositionRequest> positionRequests = new ArrayList<>();
    private final List<ControlRequest> customRequests = new ArrayList<>();
    private EncoderIOCANcoder encoder;
    private double primarySignalUpdateFreq = 100.0;
    private double secondarySignalUpdateFreq = 50.0;
    private double optimizedSignalUpdateFreq = 4.0;

    public Builder(CANBus canbus, int id, TalonFXConfiguration config) {
      this.canbus = canbus;
      this.id = id;
      this.config = config;
    }

    /**
     * Add a follower motor that matches the output of the leader motor.
     *
     * @param id The CAN ID of the follower motor.
     * @param alignment Choose whether the follower motor's direction is the same or the opposite of
     *     the leader motor.
     */
    public Builder addFollower(int id, MotorAlignmentValue alignment) {
      followers.add(Pair.of(id, alignment));
      return this;
    }

    public Builder addControlRequest(PositionVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    public Builder addControlRequest(MotionMagicVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    public Builder addControlRequest(MotionMagicExpoVoltage request) {
      positionRequests.add((leader, angle) -> leader.setControl(request.withPosition(angle)));
      return this;
    }

    /**
     * This method is designed to allow using other compatible ControlRequest types from the Phoenix
     * 6 API with this motor that are not explicitly offered in the other {@link
     * Builder#addControlRequest} methods. The slot numbers of custom control requests and callbacks
     * are zero-indexed in the order they are added.
     */
    public Builder addControlRequest(ControlRequest request) {
      customRequests.add(request);
      return this;
    }

    /**
     * This method may be used to add user-defined callbacks that require the raw motor object. The
     * slot numbers of custom control requests and callbacks are zero-indexed in the order they are
     * added.
     */
    public Builder addCallback(Consumer<TalonFX> callback) {
      customRequests.add((leader, output) -> callback.accept(leader));
      return this;
    }

    /**
     * Configure this motor to use a CANCoder as its feedback source. If the motor was already
     * configured to use the encoder, this method is not necessary.
     */
    public Builder addCANCoder(EncoderIOCANcoder encoder) {
      this.encoder = encoder;
      return this;
    }

    /**
     * Manually set the update frequency of the status signals.
     *
     * @param primaryFreqHz Frequency (in hertz) to set the primary signals used in closed loop
     *     control (position, velocity)
     * @param secondaryFreqHz Frequency (in hertz) to set the secondary signals used for logging
     *     motor status (voltage, current, temp)
     */
    public Builder setSignalUpdateFrequency(double primaryFreqHz, double secondaryFreqHz) {
      this.primarySignalUpdateFreq = primaryFreqHz;
      this.secondarySignalUpdateFreq = secondaryFreqHz;
      return this;
    }

    /**
     * Manually set the update frequency of the optimized status signals. These are signals that are
     * not being explicitly logged in AdvantageKit, but can be viewed in the hoot logs.
     */
    public Builder setOptimizedSignalFrequency(double optimizedFreqHz) {
      this.optimizedSignalUpdateFreq = optimizedFreqHz;
      return this;
    }

    public MotorIOTalonFX build() {
      int[] followerIds = new int[followers.size()];
      MotorAlignmentValue[] followerAlignments = new MotorAlignmentValue[followers.size()];
      for (int i = 0; i < followers.size(); i++) {
        followerIds[i] = followers.get(i).getFirst();
        followerAlignments[i] = followers.get(i).getSecond();
      }
      return new MotorIOTalonFX(
          canbus,
          id,
          config,
          followerIds,
          followerAlignments,
          positionRequests.toArray(new PositionRequest[0]),
          customRequests.toArray(new ControlRequest[0]),
          encoder,
          primarySignalUpdateFreq,
          secondarySignalUpdateFreq,
          optimizedSignalUpdateFreq);
    }
  }
}
