/*
 * Decision-tree model for the strategy editor. Pure functions, no DOM.
 *
 * Mirrors frc.robot.util.DecisionTree.Node. On disk (what the robot reads) a node is either
 *   leaf:      { "leafCommandId": "scoreL1" }
 *   condition: { "conditionId": "stockedUp", "yesBranch": {...}, "noBranch": {...} }
 * In the editor every node also gets a transient id so it can be selected and edited:
 *   { id, type: 'command' | 'condition', ref, yes, no }
 */
(function (global) {
  'use strict';

  // Layout metrics, shared with the renderer.
  const NODE_W = 172;
  const NODE_H = 54;
  const GAP_X = 26;
  const GAP_Y = 70;

  let nextId = 1;
  const uid = () => 'n' + nextId++;

  const leaf = (ref = 'none') => ({ id: uid(), type: 'command', ref, yes: null, no: null });
  const condition = (ref, yes = leaf(), no = leaf()) => ({
    id: uid(),
    type: 'condition',
    ref,
    yes,
    no,
  });

  /** Robot JSON -> editor tree. Same precedence as Java: a non-null leafCommandId makes a leaf. */
  function fromRobotJson(obj) {
    if (obj === null || typeof obj !== 'object' || Array.isArray(obj)) {
      throw new Error('Every node must be a JSON object');
    }
    if (obj.leafCommandId != null) return leaf(String(obj.leafCommandId));
    if (obj.conditionId == null) {
      throw new Error('A node has neither "conditionId" nor "leafCommandId"');
    }
    // A missing branch would NPE on the robot; load it as an empty leaf so validation flags it.
    return condition(
      String(obj.conditionId),
      obj.yesBranch == null ? leaf('') : fromRobotJson(obj.yesBranch),
      obj.noBranch == null ? leaf('') : fromRobotJson(obj.noBranch)
    );
  }

  /** Editor tree -> robot JSON (key order matches the Java fields). */
  function toRobotJson(node) {
    if (node.type === 'command') return { leafCommandId: node.ref };
    return {
      conditionId: node.ref,
      yesBranch: toRobotJson(node.yes),
      noBranch: toRobotJson(node.no),
    };
  }

  const serialize = (root) => JSON.stringify(toRobotJson(root), null, 2) + '\n';
  const parse = (text) => fromRobotJson(JSON.parse(text));
  /** Deep copy with fresh ids (for paste / duplicate). */
  const clone = (node) => fromRobotJson(toRobotJson(node));

  /** Pre-order traversal; yes branch before no branch. */
  function walk(node, fn, parent = null, side = null, depth = 0) {
    fn(node, parent, side, depth);
    if (node.type === 'condition') {
      walk(node.yes, fn, node, 'yes', depth + 1);
      walk(node.no, fn, node, 'no', depth + 1);
    }
  }

  /** Returns [{ node, side }] from the root down to the node with this id, or null. */
  function pathTo(root, id) {
    if (root.id === id) return [{ node: root, side: null }];
    if (root.type !== 'condition') return null;
    for (const side of ['yes', 'no']) {
      const rest = pathTo(root[side], id);
      if (rest) {
        rest[0].side = side;
        return [{ node: root, side: null }, ...rest];
      }
    }
    return null;
  }

  function find(root, id) {
    const path = pathTo(root, id);
    if (!path) return null;
    const last = path[path.length - 1];
    const parent = path.length > 1 ? path[path.length - 2].node : null;
    return { node: last.node, parent, side: last.side };
  }

  /** Swaps the node with this id for `replacement`. Returns the (possibly new) root. */
  function replace(root, id, replacement) {
    const hit = find(root, id);
    if (!hit) return root;
    if (!hit.parent) return replacement;
    hit.parent[hit.side] = replacement;
    return root;
  }

  /**
   * Top-down layout: leaves take consecutive columns, a condition sits centred over its two
   * subtrees. Subtrees own disjoint column ranges, so nodes never overlap. Positions are the
   * node's top-left corner.
   */
  function layout(root) {
    const pos = new Map();
    let nextX = 0;
    let maxDepth = 0;
    (function place(node, depth) {
      maxDepth = Math.max(maxDepth, depth);
      let x;
      if (node.type === 'condition') {
        x = (place(node.yes, depth + 1) + place(node.no, depth + 1)) / 2;
      } else {
        x = nextX;
        nextX += NODE_W + GAP_X;
      }
      pos.set(node.id, { x, y: depth * (NODE_H + GAP_Y) });
      return x;
    })(root, 0);
    return { pos, width: nextX - GAP_X, height: (maxDepth + 1) * (NODE_H + GAP_Y) - GAP_Y };
  }

  /**
   * Returns Map<nodeId, message> for nodes the robot would throw on.
   * `registry` is { commands: Map, conditions: Map } or null when IDs could not be scanned.
   */
  function validate(root, registry) {
    const issues = new Map();
    walk(root, (n) => {
      const kind = n.type;
      if (!n.ref) {
        issues.set(n.id, `Empty ${kind} ID`);
      } else if (registry) {
        const known = kind === 'command' ? registry.commands : registry.conditions;
        if (!known.has(n.ref)) {
          issues.set(n.id, `"${n.ref}" is not registered as a ${kind} in the robot code`);
        }
      }
    });
    return issues;
  }

  /** Ids visited by DecisionTree.Node.evaluate() given condition values; the last one is the leaf. */
  function trace(root, valueOf) {
    const ids = [];
    for (let n = root; n; n = n.type === 'condition' ? (valueOf(n.ref) ? n.yes : n.no) : null) {
      ids.push(n.id);
    }
    return ids;
  }

  /** Distinct non-empty refs of one node type, in tree order. */
  function usedRefs(root, type) {
    const refs = new Set();
    walk(root, (n) => {
      if (n.type === type && n.ref) refs.add(n.ref);
    });
    return [...refs];
  }

  const api = {
    NODE_W,
    NODE_H,
    leaf,
    condition,
    fromRobotJson,
    toRobotJson,
    serialize,
    parse,
    clone,
    walk,
    pathTo,
    find,
    replace,
    layout,
    validate,
    trace,
    usedRefs,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else global.StrategyModel = api;
})(this);
