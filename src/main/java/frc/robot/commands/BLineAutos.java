package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotUtil;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class BLineAutos {
  private static final String STARTING_PATH_PREFIX = "s_";

  private static FollowPath.Builder builder;

  private static Drive drive;

  public static void configure(Drive drive) {
    BLineAutos.drive = drive;

    builder =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(2.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.4),
                new PIDController(0.2, 0.0, 0.0))
            .withDefaultShouldFlip()
            // in this field, alliance sides are mirrored from each other
            .withShouldMirror(RobotUtil::isRedAlliance)
            .withTRatioBasedTranslationHandoffs(true);

    FollowPath.setPoseLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    //    FollowPath.setTranslationListLoggingConsumer(
    //        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
  }

  public static List<Pair<String, Command>> getAllPathCommands() {
    List<String> pathNames = getAllPathNames();
    List<Pair<String, Command>> commands = new ArrayList<>();
    for (String pathName : pathNames) {
      Path path = new Path(pathName);
      if (pathName.startsWith(STARTING_PATH_PREFIX)) {
        pathName = pathName.substring(STARTING_PATH_PREFIX.length());
        commands.add(Pair.of(pathName + "ResetPose", resetPose(path)));
      }
      commands.add(Pair.of(pathName, Commands.defer(() -> builder.build(path), Set.of(drive))));
    }
    return commands;
  }

  private static Command resetPose(Path path) {
    return Commands.runOnce(
        () -> {
          Pose2d startPose = path.getStartPose();
          if (RobotUtil.isRedAlliance()) {
            startPose = FlippingUtil.flipFieldPose(startPose);
            startPose = FlippingUtil.mirrorFieldPose(startPose);
          }
          drive.setPose(startPose);
        });
  }

  private static List<String> getAllPathNames() {
    File[] autoFiles = new File(Filesystem.getDeployDirectory(), "autos/paths").listFiles();
    if (autoFiles == null) {
      return new ArrayList<>();
    }
    return Stream.of(autoFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".json"))
        .map(name -> name.substring(0, name.lastIndexOf(".")))
        .toList();
  }
}
