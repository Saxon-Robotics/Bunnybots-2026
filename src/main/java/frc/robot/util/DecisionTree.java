package frc.robot.util;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import lombok.Getter;

public class DecisionTree {
  private DecisionTree() {
    /* This utility class should not be instantiated */
  }

  @JsonInclude(JsonInclude.Include.NON_NULL)
  public static class Node {
    @Getter private final String conditionId;
    @Getter private final Node yesBranch;
    @Getter private final Node noBranch;
    @Getter private final String leafCommandId;

    // Unified Constructor for Jackson Data-Binding
    @JsonCreator
    public Node(
        @JsonProperty("conditionId") String conditionId,
        @JsonProperty("yesBranch") Node yesBranch,
        @JsonProperty("noBranch") Node noBranch,
        @JsonProperty("leafCommandId") String leafCommandId) {
      this.conditionId = conditionId;
      this.yesBranch = yesBranch;
      this.noBranch = noBranch;
      this.leafCommandId = leafCommandId;
    }

    public boolean isLeaf() {
      return leafCommandId != null;
    }

    // Pass registers into the evaluation loop
    public Command evaluate() {
      if (this.isLeaf()) {
        Command command = commandRegistry.get(this.leafCommandId);
        if (command == null)
          throw new IllegalStateException("Unknown command ID: " + leafCommandId);
        return command;
      }

      BooleanSupplier supplier = conditionRegistry.get(this.conditionId);
      if (supplier == null) throw new IllegalStateException("Unknown condition ID: " + conditionId);

      if (supplier.getAsBoolean()) {
        return yesBranch.evaluate();
      } else {
        return noBranch.evaluate();
      }
    }
  }

  private static final Map<String, BooleanSupplier> conditionRegistry = new HashMap<>();
  private static final Map<String, Command> commandRegistry = new HashMap<>();
  private static final ObjectMapper mapper = new ObjectMapper();
  private static final File STRATEGY_DIRECTORY =
      new File(Filesystem.getDeployDirectory(), "strategies");

  public static final Node EMPTY = new Node(null, null, null, "none");

  static {
    commandRegistry.put("none", Commands.none());
  }

  /** Loads a strategy file (e.g. "Example.json") from deploy/strategies. */
  public static Node fromJson(String fileName) throws IOException {
    return mapper.readValue(new File(STRATEGY_DIRECTORY, fileName), Node.class);
  }

  public static List<String> getAllStrategyNames() {
    File[] autoFiles = STRATEGY_DIRECTORY.listFiles();
    if (autoFiles == null) {
      return new ArrayList<>();
    }
    return Stream.of(autoFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".json"))
        .toList();
  }

  public static void registerCondition(String name, BooleanSupplier condition) {
    conditionRegistry.put(name, condition);
  }

  public static void registerCommand(String name, Command command) {
    commandRegistry.put(name, command);
  }

  public static BooleanSupplier getCondition(String id) {
    return conditionRegistry.get(id);
  }

  public static Command getCommand(String id) {
    return commandRegistry.get(id);
  }
}
