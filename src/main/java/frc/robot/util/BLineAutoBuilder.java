package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class BLineAutoBuilder {
  private static Map<Command, Path> commandToPath;
  private static FollowPath.Builder builder;

  public static void configure(FollowPath.Builder newBuilder) {
    builder = newBuilder;
  }

  public static void addPathsToChooser(LoggedDashboardChooser<Command> autoChooser) {
    if (builder == null) return;

    commandToPath = new HashMap<>();

    for (String name : getAllPathNames()) {
      if (name.startsWith(BetterAutoChooser.FLIP_PREFIX)) {
        // grab path from name
        Path leftPath = new Path(name);
        Path rightPath = leftPath.copy();
        // flip the correct path
        if (BetterAutoChooser.RIGHT_IS_FLIPPED) {
          rightPath.mirror();
        } else {
          leftPath.mirror();
        }
        // build autos
        Command leftPathCommand = builder.build(leftPath);
        Command rightPathCommand = builder.build(rightPath);
        // add autos to chooser
        String remainder = name.substring(BetterAutoChooser.FLIP_PREFIX.length());
        autoChooser.addOption("Left " + remainder + " (Generated)", leftPathCommand);
        autoChooser.addOption("Right " + remainder + " (Generated)", rightPathCommand);
        // map autos to paths for Field2d visualization
        commandToPath.put(leftPathCommand, rightPath);
        commandToPath.put(rightPathCommand, leftPath);
      } else {
        // no flip needed
        Path path = new Path(name);
        Command pathCommand = builder.build(path);
        autoChooser.addOption(name, pathCommand);
        commandToPath.put(pathCommand, path);
      }
    }
  }

  public static Path getPathFromCommand(Command pathCommand) {
    return commandToPath.get(pathCommand);
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
