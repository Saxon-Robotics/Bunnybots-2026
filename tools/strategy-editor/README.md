# Strategy Editor

A visual editor for the full-auto decision trees in `src/main/deploy/strategies/*.json`,
loaded on the robot by `frc.robot.util.DecisionTree` and run by `SelfDriving`.

## Running it

Open `index.html` in **Chrome or Edge**, click **Open robot project…** and pick the repo root
(the folder with `build.gradle`). Next time, **Reopen** skips the folder picker.

If the folder button does nothing when the page is opened as a file, serve it locally instead:

```bash
python -m http.server 8765 --directory tools/strategy-editor
```

and browse to http://localhost:8765. Other browsers (Firefox) get an import/download fallback.

## How it maps to the robot code

| Editor | JSON | Robot |
| --- | --- | --- |
| **IF** node | `{"conditionId", "yesBranch", "noBranch"}` | `DecisionTree.registerCondition("id", BooleanSupplier)` |
| **RUN** node | `{"leafCommandId"}` | `DecisionTree.registerCommand("id", Command)` |

The palette lists every ID found by scanning `src/main/java` for
`DecisionTree.registerCommand("…")` / `DecisionTree.registerCondition("…")` (the literal
`DecisionTree.` prefix is required). Unregistered IDs are flagged, because the robot throws
`Unknown … ID` when evaluation reaches them. The palette rescans when the window regains focus.

`SelfDriving.routine()` re-evaluates the tree every time the chosen command finishes. The
**Preview** tab lets you set condition values and shows which command would be picked.

## Shortcuts

| Key | Action |
| --- | --- |
| Ctrl+S | Save |
| Ctrl+Z / Ctrl+Y | Undo / redo |
| Arrows | Up = parent, Left = yes branch, Right = no branch |
| Delete | Replace the selected branch with `none` |
| Ctrl+C / Ctrl+V | Copy a branch / paste it over the selected node |
| F | Fit the tree to the view |
| Esc | Deselect |

Mouse: drag the background to pan, scroll to zoom. Drag a command or condition from the palette
onto a node. A command replaces the node. A condition dropped on a command wraps it in an IF.
