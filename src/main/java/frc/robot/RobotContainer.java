// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControlScheme;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.BLineAutos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SelfDriving;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.objectivetracker.ObjectiveTracker;
import frc.robot.subsystems.trader.Trader;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AutoAlign;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.RobotUtil;
import frc.robot.util.io.GuitarHeroController;
import frc.robot.util.sim.Arena2026Bunnybots;
import frc.robot.util.sim.HarvestHavocCarrotOnFly;
import frc.robot.util.sim.SimulationManager;
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
  private final Command teleopFullAuto;

  // Simulated things
  private final SwerveDriveSimulation driveSimulation;
  private final SimulationManager sim;

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
        vision = new Vision(drive, new VisionIO() {});
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_OFFSET),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_OFFSET),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_2_NAME, VisionConstants.CAMERA_2_OFFSET),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_3_NAME, VisionConstants.CAMERA_3_OFFSET));
        elevator = new Elevator();
        gripper = new Gripper();
        trader = new Trader();
        sim = null;
      }
      case SIM -> {
        Arena2026Bunnybots arena = new Arena2026Bunnybots();
        SimulatedArena.overrideInstance(arena);
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
            SimulationManager.createInstance(
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
    controlProfiles.addOption("Testing", ControlScheme.TEST);
    controlProfiles.onChange(this::setControlScheme);

    // Set up commands for PathPlanner & BLine
    configureAutoCommands();

    // Set up BLine paths
    addBLinePaths();

    // Set up full autos
    SelfDriving selfDriving =
        new SelfDriving(
            ObjectiveTracker.initialize(drive, vision, elevator, gripper, trader),
            drive,
            elevator,
            gripper,
            trader);

    LoggedDashboardChooser<String> strategyChooser =
        new LoggedDashboardChooser<>("Full Auto Strategy");
    selfDriving.setUpChooser(strategyChooser);

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());
    autoChooser.addOption("Full Self Driving", selfDriving.routine());
    teleopFullAuto = selfDriving.routine().withName("Teleop Full Self Driving");

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

    // Visualize BLine Paths
    //    autoChooser.onChange(
    //        command -> {
    //          Path path = BLineAutoBuilder.getPathFromCommand(command);
    //          BLineField.drawPath(drive.field, "path", path != null ? path : new Path());
    //        });

    DriverStation.silenceJoystickConnectionWarning(true);

    Logger.recordOutput("Field/BlueOven", Constants.FieldConstants.BLUE_OVEN);
    Logger.recordOutput("Field/RedOven", Constants.FieldConstants.RED_OVEN);

    SmartDashboard.putData(CommandScheduler.getInstance());
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
    // Default command, normal field-relative drive
    Command defaultDriveCommand =
        DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX())
            .withName("Default Drive");
    // Lock wheels to X pattern
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);
    Command autoAlign =
        DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget).withName("Auto Align");
    // Auto align to pantry (locked angle and y)
    Command pantryAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.PANTRY, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget))
            .andThen(
                DriveCommands.singleAxisJoystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> AutoAlign.getLastTargetPose().getY(),
                        // avoid recomputing nearest pantry
                        () -> AutoAlign.getLastTargetPose().getRotation())
                    .beforeStarting(() -> drive.setSpeedLimiter(true))
                    .finallyDo(() -> drive.setSpeedLimiter(false)))
            .withName("Pantry Align");
    // Set auto align targets
    Command ovenAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.OVEN, drive.getPose()));
    Command rampAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.RAMP, drive.getPose()));
    Command depotAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.DEPOT, drive.getPose()));
    Command tableAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.TABLE, drive.getPose()));

    /* Elevator commands */
    DoubleSupplier elevatorJoystick =
        () ->
            Math.copySign(
                Math.pow(
                    MathUtil.applyDeadband(
                        operatorController.getLeftY(), ControllerConstants.OPERATOR_DEADBAND),
                    2),
                -operatorController.getLeftY());
    Command manualElevator = elevator.manualControl(elevatorJoystick).withName("Manual Elevator");
    Command elevatorHoming = elevator.homingSequence().withName("Elevator Homing");
    Command stowElevator = elevator.stow().withName("Stow Elevator");
    Command rampElevator = elevator.ramp().withName("Ramp Elevator");
    Command l1Elevator = elevator.l1().withName("L1 Elevator");
    Command l2Elevator = elevator.l2().withName("L2 Elevator");
    Command l3Elevator = elevator.l3().withName("L3 Elevator");
    // emergency disable while true
    Command disableElevator =
        elevator
            .release()
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .ignoringDisable(true)
            .withName("Disable Elevator");

    /* End effector commands */
    Command gripperIntake = gripper.intake().withName("Gripper Intake");
    Command gripperEject = gripper.eject().withName("Gripper Eject");

    /* Trader commands */
    Command traderEject = trader.eject().withName("Trader Eject");
    Command traderReverse = trader.reverse().withName("Trader Reverse");

    drive.setDefaultCommand(defaultDriveCommand);

    // elevator override
    new Trigger(() -> elevatorJoystick.getAsDouble() != 0.0).whileTrue(manualElevator);

    /* driver controls */
    driverController.x().whileTrue(lockWheels);
    driverController.y().onTrue(zeroGyro);
    driverController.a().onTrue(tableAlign).whileTrue(autoAlign);
    BooleanSupplier isPossessing = () -> gripper.isLoaded() || trader.isLoaded();
    Command ovenOrRampAlign = new ConditionalCommand(ovenAlign, rampAlign, isPossessing);
    driverController.leftBumper().onTrue(ovenOrRampAlign).whileTrue(autoAlign);
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
    operatorController.leftBumper().onTrue(rampElevator).whileTrue(rumbleCommand);
    operatorController.povLeft().onTrue(l1Elevator).whileTrue(rumbleCommand);
    operatorController.povRight().onTrue(l2Elevator).whileTrue(rumbleCommand);
    operatorController.povUp().onTrue(l3Elevator).whileTrue(rumbleCommand);
    operatorController.rightBumper().onTrue(elevatorHoming).whileTrue(rumbleCommand);
    operatorController.leftTrigger(0.85).whileTrue(disableElevator);

    operatorController.b().whileTrue(gripperIntake);
    operatorController.y().whileTrue(gripperEject);

    operatorController.a().whileTrue(traderEject);
    operatorController.x().whileTrue(traderReverse);
    // test mode (single controller)
    BooleanSupplier testProfile = () -> controlScheme == ControlScheme.TEST;
    /* todo implementation */

    /* keyboard controls for sim */
    if (currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(3);

      // superstructure keybinds
      keyboard.button(1).onTrue(l1Elevator);
      keyboard.button(2).onTrue(l2Elevator);
      keyboard.button(3).onTrue(l3Elevator);
      keyboard.button(4).onTrue(rampElevator);
      keyboard.button(5).onTrue(stowElevator);
      keyboard.button(6).whileTrue(gripperIntake);
      keyboard.button(7).whileTrue(gripperEject);
      keyboard.button(8).whileTrue(traderEject);

      keyboard.button(12).onTrue(ovenOrRampAlign).whileTrue(autoAlign);
      keyboard.button(13).and(isPossessing).whileTrue(pantryAlign);
      keyboard
          .button(14)
          .and(new Trigger(isPossessing).negate())
          .onTrue(depotAlign)
          .whileTrue(autoAlign);

      keyboard.button(13).onTrue(tableAlign).whileTrue(autoAlign);
      // drop carrots
      keyboard
          .button(9)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_RAMP
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_RAMP)));
      keyboard
          .button(10)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_REAR_DEPOT
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_REAR_DEPOT)));
      keyboard
          .button(11)
          .onTrue(
              Commands.runOnce(
                  () ->
                      sim.dropCarrot(
                          RobotUtil.isRedAlliance()
                              ? HarvestHavocCarrotOnFly.CarrotStations.RED_SIDE_DEPOT
                              : HarvestHavocCarrotOnFly.CarrotStations.BLUE_SIDE_DEPOT)));
    }
  }

  private void configureAutoCommands() {
    Command pantryAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.PANTRY, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));
    Command ovenAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.OVEN, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));
    Command rampAlign =
        Commands.runOnce(() -> AutoAlign.setTarget(AutoAlign.Target.RAMP, drive.getPose()))
            .andThen(DriveCommands.alignToTarget(drive, AutoAlign::getLastAPTarget));

    Command scorePantry =
        Commands.sequence(pantryAlign, gripper.eject().until(() -> !gripper.isLoaded()));
    Command scoreOven =
        Commands.sequence(ovenAlign, trader.eject().until(() -> !trader.isLoaded()));
    Command intakeFromRamp =
        Commands.sequence(rampAlign, gripper.intake().until(gripper::isLoaded));
    if (Constants.currentMode == Constants.Mode.SIM) {
      intakeFromRamp =
          intakeFromRamp.alongWith(
              Commands.runOnce(
                  () ->
                      SimulationManager.getInstance()
                          .dropCarrot(
                              RobotUtil.isRedAlliance()
                                  ? HarvestHavocCarrotOnFly.CarrotStations.RED_RAMP
                                  : HarvestHavocCarrotOnFly.CarrotStations.BLUE_RAMP)));
    }

    NamedCommands.registerCommand("ScorePantry", scorePantry);
    NamedCommands.registerCommand("ScoreOven", scoreOven);
    NamedCommands.registerCommand("IntakeRamp", intakeFromRamp);

    FollowPath.registerEventTrigger("ElevatorStow", elevator.stow());
    FollowPath.registerEventTrigger("ElevatorRamp", elevator.ramp());
    FollowPath.registerEventTrigger("ElevatorL1", elevator.l1());
    FollowPath.registerEventTrigger("ElevatorL2", elevator.l2());
    FollowPath.registerEventTrigger("ElevatorL3", elevator.l3());
  }

  private void addBLinePaths() {
    BLineAutos.configure(drive);

    NamedCommands.registerCommands(BLineAutos.getAllPathCommands());
  }

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
      Command manualElevatorUp = elevator.manualControl(() -> 0.8).withName("Manual Elevator");
      Command manualElevatorDown = elevator.manualControl(() -> -0.7).withName("Manual Elevator");
      Command elevatorHoming = elevator.homingSequence().withName("Elevator Homing");
      Command stowElevator = elevator.stow().withName("Stow Elevator");
      Command rampElevator = elevator.ramp().withName("Ramp Elevator");
      Command l1Elevator = elevator.l1().withName("L1 Elevator");
      Command l2Elevator = elevator.l2().withName("L2 Elevator");
      Command l3Elevator = elevator.l3().withName("L3 Elevator");
      Command disableElevator =
          elevator
              .release()
              .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
              .ignoringDisable(true)
              .withName("Disable Elevator");

      /* End effector commands */
      Command gripperIntake = gripper.intake().withName("Gripper Intake");
      Command gripperEject = gripper.eject().withName("Gripper Eject");

      /* Trader commands */
      Command traderReverse = trader.reverse().withName("Trader Reverse");
      Command traderEject = trader.eject().withName("Trader Eject");

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
      guitarHeroController.orange().and(guitarHeroControls).and(neutralStrumBar).onTrue(l3Elevator);
      guitarHeroController
          .green()
          .and(guitarHeroControls)
          .and(upStrumBar)
          .whileTrue(manualElevatorUp);
      guitarHeroController
          .red()
          .and(guitarHeroControls)
          .and(upStrumBar)
          .whileTrue(manualElevatorDown);
      guitarHeroController.blue().and(guitarHeroControls).and(upStrumBar).onTrue(elevatorHoming);
      guitarHeroController
          .orange()
          .and(guitarHeroControls)
          .and(upStrumBar)
          .whileTrue(disableElevator);

      guitarHeroController
          .green()
          .and(guitarHeroControls)
          .and(downStrumBar)
          .whileTrue(gripperIntake);
      guitarHeroController.red().and(guitarHeroControls).and(downStrumBar).whileTrue(gripperEject);
      guitarHeroController
          .blue()
          .and(guitarHeroControls)
          .and(downStrumBar)
          .whileTrue(traderReverse);
      guitarHeroController
          .orange()
          .and(guitarHeroControls)
          .and(downStrumBar)
          .whileTrue(traderEject);
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

  /**
   * @return the full self driving routine to run during teleop
   */
  public Command getTeleopCommand() {
    return teleopFullAuto;
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    sim.preloadTrader();
  }

  public void simulationInit() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    if (!DriverStation.isAutonomous()) {
      sim.undoTraderPreload();
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    sim.simulationPeriodic();
  }
}
