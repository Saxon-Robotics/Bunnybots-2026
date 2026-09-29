package frc.robot.subsystems.trader;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class TraderConstants {
  public static final double RPS = 50;
  public static final double STERILIZATION_RPS = 15;

  public static final double MOI = 0.002;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.12).withKA(0));

  public static final double BEAMBREAK_THRESHOLD = 75; // mm

  public static final double CARROTS_PER_SEC = 1.0; // sim only
}
