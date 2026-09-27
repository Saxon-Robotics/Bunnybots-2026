// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FieldUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum ControlScheme {
    MAIN,
    TEST,
    GUITAR_HERO
  }

  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int GUITAR_HERO_CONTROLLER_PORT = 2;
    public static final double DRIVER_DEADBAND = 0.1;
    public static final double OPERATOR_DEADBAND = 0.1;
    public static final double GUITAR_HERO_DEADBAND = 0.1;
  }

  public static final class CANConstants {
    // superstructure canbus
    public static final CANBus SUPERSTRUCTURE = new CANBus("Superstructure");
    public static final int ELEVATOR_LEFT = 1;
    public static final int ELEVATOR_RIGHT = 2;
    public static final int GRIPPER_LEFT = 3;
    public static final int GRIPPER_RIGHT = 4;
    public static final int TRADER_LEFT = 5;
    public static final int TRADER_RIGHT = 6;
    // rio canbus
    public static final int GRIPPER_LASERCAN = 20;
    public static final int TRADER_LASERCAN = 21;
  }

  public static final class FieldConstants {
    public static final Distance FIELD_LENGTH = Meters.of(Units.feetToMeters(54));
    public static final Distance FIELD_WIDTH = Meters.of(Units.feetToMeters(27));
    public static final Translation2d ORIGIN =
        new Translation2d(FIELD_LENGTH.div(2.0), FIELD_WIDTH.div(2.0));

    public static final Pose2d BLUE_PANTRY =
        new Pose2d(new Translation2d(-6.734, 3.557).plus(ORIGIN), Rotation2d.kCCW_90deg);
    public static final Pose2d RED_PANTRY =
        new Pose2d(new Translation2d(6.734, 3.557).plus(ORIGIN), Rotation2d.kCCW_90deg);

    public static final Pose2d BLUE_OVEN =
        new Pose2d(new Translation2d(-7.8135, -1.422).plus(ORIGIN), Rotation2d.k180deg);
    public static final Pose2d RED_OVEN =
        new Pose2d(new Translation2d(7.8135, -1.422).plus(ORIGIN), Rotation2d.k180deg);

    public static final Pose2d BLUE_RAMP =
        new Pose2d(new Translation2d(-6.013, -1.739).plus(ORIGIN), Rotation2d.k180deg);
    public static final Pose2d RED_RAMP =
        new Pose2d(new Translation2d(6.013, -1.739).plus(ORIGIN), Rotation2d.kZero);

    public static final Pose2d BLUE_REAR_DEPOT = new Pose2d();
    public static final Pose2d BLUE_SIDE_DEPOT = new Pose2d();
    public static final Pose2d RED_REAR_DEPOT = FieldUtils.allianceRelativeFlip(BLUE_REAR_DEPOT);
    public static final Pose2d RED_SIDE_DEPOT = FieldUtils.allianceRelativeFlip(BLUE_SIDE_DEPOT);

    public static final Pose2d BLUE_TABLE_ZONE = new Pose2d();
    public static final Pose2d RED_TABLE_ZONE = new Pose2d();
  }
}
