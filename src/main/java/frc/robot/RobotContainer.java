// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControlScheme;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import frc.robot.util.io.GuitarHeroController;
import frc.robot.util.sim.Arena2026Bunnybots;
import frc.robot.util.sim.HarvestHavocCarrotOnFly;
import frc.robot.util.sim.SimulationHelper;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Gripper gripper;
  private final Trader trader;

  // controllers
  private ControlScheme controlScheme = ControlScheme.MAIN;
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  private GuitarHeroController guitarHeroController;

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulated things
  private final SwerveDriveSimulation driveSimulation;
  private final SimulationHelper sim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL -> {
        driveSimulation = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight, false),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.BackRight, false),
                pose -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_2_NAME, VisionConstants.CAMERA_2_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_3_NAME, VisionConstants.CAMERA_3_OFFSET));
        elevator = new Elevator();
        gripper = new Gripper();
        trader = new Trader();
        sim = null;
      }
      case SIM -> {
        Arena2026Bunnybots arena = new Arena2026Bunnybots();
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().resetFieldForAuto();
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.CAMERA_0_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.CAMERA_1_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_2_NAME,
                    VisionConstants.CAMERA_2_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_3_NAME,
                    VisionConstants.CAMERA_3_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose));
        elevator = new Elevator();
        gripper = new Gripper();
        trader = new Trader();
        sim =
            SimulationHelper.createInstance(
                elevator, trader, gripper, driveSimulation, drive::getChassisSpeeds);
      }
      default -> {
        /* REPLAY */
        driveSimulation = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                pose -> {});
        vision =
            new Vision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator();
        gripper = new Gripper();
        trader = new Trader();
        sim = null;
      }
    }

    // Configure the trigger bindings
    configureBindings();

    LoggedDashboardChooser<ControlScheme> controlProfiles =
        new LoggedDashboardChooser<>("Control Profile");
    controlProfiles.addDefaultOption("Main", ControlScheme.MAIN);
    controlProfiles.addOption("Guitar Hero Operator", ControlScheme.GUITAR_HERO);
    //    controlProfiles.addOption("Guitar Hero Full Control", ControlScheme.GUITAR_HERO_FULL);
    controlProfiles.addOption("Testing", ControlScheme.TEST);
    controlProfiles.onChange(this::setControlScheme);

    // Set up commands for PathPlanner
    configureAutoCommands();

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());

    // Set up SysId routines when not in competition
    if (!DriverStation.isFMSAttached()) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // Set up custom autos (non-PathPlanner)
    //    autoChooser.addOption("Full System Check", Autos.systemCheck(drive, shooter, feeder,
    // intake));

    DriverStation.silenceJoystickConnectionWarning(true);

    Logger.recordOutput("Field/BlueOven", Constants.FieldConstants.BLUE_OVEN);
    Logger.recordOutput("Field/RedOven", Constants.FieldConstants.RED_OVEN);
    Logger.recordOutput("Field/BluePantry", Constants.FieldConstants.BLUE_PANTRY);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RobotUtil.setDriverController(driverController);
    RobotUtil.setOperatorController(operatorController);

    /* Drive commands */
    // Lock wheels to X pattern
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);
    Command autoAlign = DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget);
    // Auto align to pantry (locked angle and y)
    Command pantryAlign =
        Commands.runOnce(() -> AutoAlign.getTargetPose(AutoAlign.Target.PANTRY, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget))
            .andThen(
                DriveCommands.singleAxisJoystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> AutoAlign.getLastTarget().getY(),
                        // avoid recomputing nearest pantry
                        () -> AutoAlign.getLastTarget().getRotation())
                    .beforeStarting(() -> drive.setSpeedLimiter(true))
                    .finallyDo(() -> drive.setSpeedLimiter(false)));
    // Set auto align targets
    Command ovenAlign =
        Commands.runOnce(() -> AutoAlign.getTargetPose(AutoAlign.Target.OVEN, drive.getPose()));
    Command rampAlign =
        Commands.runOnce(() -> AutoAlign.getTargetPose(AutoAlign.Target.RAMP, drive.getPose()));
    Command depotAlign =
        Commands.runOnce(() -> AutoAlign.getTargetPose(AutoAlign.Target.DEPOT, drive.getPose()));
    Command tableAlign =
        Commands.runOnce(() -> AutoAlign.getTargetPose(AutoAlign.Target.TABLE, drive.getPose()));

    /* Elevator commands */
    DoubleSupplier elevatorJoystick =
        () ->
            Math.copySign(
                Math.pow(
                    MathUtil.applyDeadband(
                        operatorController.getLeftY(), ControllerConstants.OPERATOR_DEADBAND),
                    2),
                -operatorController.getLeftY());
    Command manualElevator = elevator.manualControl(elevatorJoystick);
    Command elevatorHoming = elevator.homingSequence();
    Command stowElevator = elevator.stow();
    Command rampElevator = elevator.ramp();
    Command l1Elevator = elevator.l1();
    Command l2Elevator = elevator.l2();
    // emergency disable while true
    Command disableElevator =
        elevator.release().withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);

    /* End effector commands */
    Command gripperIntake = gripper.intake();
    Command gripperEject = gripper.eject();

    /* Trader commands */
    Command traderIntake = trader.intake();
    Command traderEject = trader.eject();

    // Default command, normal field-relative drive
    Command defaultDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX());
    drive.setDefaultCommand(defaultDriveCommand);

    // elevator override
    new Trigger(() -> elevatorJoystick.getAsDouble() != 0.0).whileTrue(manualElevator);

    /* driver controls */
    driverController.x().whileTrue(lockWheels);
    driverController.y().onTrue(zeroGyro);
    driverController.a().onTrue(tableAlign).whileTrue(autoAlign);
    BooleanSupplier isPossessing = () -> gripper.isLoaded() || trader.isLoaded();
    driverController
        .leftBumper()
        .onTrue(new ConditionalCommand(ovenAlign, rampAlign, isPossessing))
        .whileTrue(autoAlign);
    driverController.rightBumper().and(isPossessing).whileTrue(pantryAlign);
    driverController
        .rightBumper()
        .and(new Trigger(isPossessing).negate())
        .onTrue(depotAlign)
        .whileTrue(autoAlign);

    /* operator controls */
    RobotUtil.RumbleRequest elevatorRumble = new RobotUtil.RumbleRequest(0.8, 0);
    Command rumbleCommand =
        Commands.startEnd(
            () -> RobotUtil.requestOperatorRumble(elevatorRumble),
            () -> RobotUtil.stopOperatorRumble(elevatorRumble));
    // controller vibrates when elevator buttons are pressed
    operatorController.povDown().onTrue(stowElevator).whileTrue(rumbleCommand);
    operatorController.povRight().onTrue(rampElevator).whileTrue(rumbleCommand);
    operatorController.povLeft().onTrue(l1Elevator).whileTrue(rumbleCommand);
    operatorController.povUp().onTrue(l2Elevator).whileTrue(rumbleCommand);
    operatorController.rightBumper().onTrue(elevatorHoming).whileTrue(rumbleCommand);
    operatorController.leftTrigger(0.85).whileTrue(disableElevator);

    operatorController.b().whileTrue(gripperIntake);
    operatorController.y().whileTrue(gripperEject);

    operatorController.a().whileTrue(traderIntake);
    operatorController.x().whileTrue(traderEject);
    // test mode (single controller)
    BooleanSupplier testProfile = () -> controlScheme == ControlScheme.TEST;
    /* todo implementation */

    /* keyboard controls for sim */
    if (currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(3);

      // superstructure keybinds

      // drop carrots
      keyboard
          .button(7)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_RAMP
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_RAMP)));
      keyboard
          .button(8)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_REAR_DEPOT
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_REAR_DEPOT)));
      keyboard
          .button(9)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_SIDE_DEPOT
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_SIDE_DEPOT)));
      keyboard.button(10).whileTrue(pantryAlign);
      keyboard.button(6).whileTrue(autoAlign);
    }
  }

  private void configureAutoCommands() {}

  public void setControlScheme(ControlScheme newScheme) {
    if (newScheme == ControlScheme.GUITAR_HERO) {
      configureGuitarHeroController();
    }
    controlScheme = newScheme;
  }

  private void configureGuitarHeroController() {
    if (guitarHeroController == null) {
      // lazy instantiation
      guitarHeroController =
          new GuitarHeroController(ControllerConstants.GUITAR_HERO_CONTROLLER_PORT);

      // configure triggers only once
      /* Elevator commands */
      DoubleSupplier elevatorJoystick =
          // scale tilt axis to [-1, 1]
          () -> Math.fma(200.0 / 81.0, guitarHeroController.getTiltAxis(), -119.0 / 81.0);
      Command manualElevator = elevator.manualControl(elevatorJoystick);
      Command elevatorHoming = elevator.homingSequence();
      Command stowElevator = elevator.stow();
      Command rampElevator = elevator.ramp();
      Command l1Elevator = elevator.l1();
      Command l2Elevator = elevator.l2();

      /* End effector commands */
      Command gripperIntake = gripper.intake();
      Command gripperEject = gripper.eject();

      /* Trader commands */
      Command traderIntake = trader.intake();
      Command traderEject = trader.eject();

      // controls are only active during the correct mode
      BooleanSupplier guitarHeroControls = () -> controlScheme == ControlScheme.GUITAR_HERO;
      // devious strum bar combinations
      BooleanSupplier upStrumBar = guitarHeroController.povUp();
      BooleanSupplier downStrumBar = guitarHeroController.povDown();
      BooleanSupplier neutralStrumBar = guitarHeroController.povCenter();

      guitarHeroController
          .green()
          .and(guitarHeroControls)
          .and(neutralStrumBar)
          .onTrue(stowElevator);
      guitarHeroController.red().and(guitarHeroControls).and(neutralStrumBar).onTrue(rampElevator);
      guitarHeroController.yellow().and(guitarHeroControls).and(neutralStrumBar).onTrue(l1Elevator);
      guitarHeroController.blue().and(guitarHeroControls).and(neutralStrumBar).onTrue(l2Elevator);
      guitarHeroController
          .orange()
          .and(guitarHeroControls)
          .and(neutralStrumBar)
          .onTrue(elevatorHoming);
      guitarHeroController
          .orange()
          .and(guitarHeroControls)
          .and(upStrumBar)
          .whileTrue(manualElevator);

      guitarHeroController.green().and(guitarHeroControls).and(upStrumBar).whileTrue(gripperIntake);
      guitarHeroController.red().and(guitarHeroControls).and(upStrumBar).whileTrue(gripperEject);

      guitarHeroController
          .green()
          .and(guitarHeroControls)
          .and(downStrumBar)
          .whileTrue(traderIntake);
      guitarHeroController.red().and(guitarHeroControls).and(downStrumBar).whileTrue(traderEject);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode == Constants.Mode.REAL) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode == Constants.Mode.REAL) return;

    sim.simulationPeriodic();
  }
}
