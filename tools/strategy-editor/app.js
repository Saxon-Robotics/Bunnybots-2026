/*
 * Strategy editor UI. State lives in `state`; every change goes through edit() (undoable) and
 * then render(), which redraws the sidebar, toolbar, canvas and side panel from scratch. Trees
 * are small (tens of nodes), so a full redraw is simpler than diffing and still instant.
 */
(function () {
  'use strict';

  const M = window.StrategyModel;
  const P = window.StrategyProject;
  const DRAG_MIME = 'application/x-strategy-item';
  const MAX_UNDO = 200;

  const $ = (id) => document.getElementById(id);
  const ESCAPES = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' };
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ESCAPES[c]);

  const state = {
    project: null,
    registry: null, // { commands, conditions } Map<id, file>, or null if not scannable
    files: [],
    current: null, // { name, root, saved } where saved is the last serialized-on-disk text
    selectedId: null,
    issues: new Map(),
    undo: [],
    redo: [],
    editSession: 0, // bumps whenever typing into an ID box should start a new undo step
    clipboard: null, // robot JSON of a copied subtree
    preview: new Map(), // conditionId -> boolean
    tab: 'inspect',
    view: { x: 0, y: 0, k: 1 },
    lastFolder: null,
    dirty: false, // cached isDirty() for the current render
  };

  // ------------------------------------------------------------------ helpers

  function toast(message, kind = '') {
    const el = document.createElement('div');
    el.className = 'toast ' + kind;
    el.textContent = message;
    $('toasts').append(el);
    setTimeout(() => el.remove(), kind === 'error' ? 6000 : 3500);
  }

  const isDirty = () => !!state.current && M.serialize(state.current.root) !== state.current.saved;

  function confirmDiscard() {
    return !isDirty() || confirm(`Discard unsaved changes to "${state.current.name}"?`);
  }

  function askName(title, initial = '') {
    const raw = prompt(title, initial);
    if (raw == null) return null;
    const name = raw.trim().replace(/\.json$/i, '');
    if (!name || /[\\/:*?"<>|]/.test(name)) {
      toast('Invalid file name', 'error');
      return null;
    }
    return name;
  }

  async function run(task) {
    try {
      await task();
    } catch (e) {
      if (e.name === 'AbortError') return; // user closed a picker
      console.error(e);
      toast(e.message || String(e), 'error');
    }
  }

  /** IDs offered for a node type: registered ones plus any already used in this tree. */
  function knownIds(type) {
    const registered = state.registry
      ? [...(type === 'command' ? state.registry.commands : state.registry.conditions).keys()]
      : type === 'command'
        ? ['none']
        : [];
    const used = state.current ? M.usedRefs(state.current.root, type) : [];
    return [...new Set([...registered, ...used])];
  }

  /** File that registers the ID, null if unregistered, undefined if IDs could not be scanned. */
  function registrationOf(type, ref) {
    if (!state.registry) return undefined;
    return (type === 'command' ? state.registry.commands : state.registry.conditions).get(ref) || null;
  }

  function refStatusHtml(type, ref) {
    const where = registrationOf(type, ref);
    if (!ref) return '<div class="status warn">⚠ Empty ID. The robot throws when it reaches this node.</div>';
    if (where) return `<div class="status ok">✓ Registered in ${esc(where)}</div>`;
    if (where === undefined) return '';
    const fn = type === 'condition' ? 'registerCondition' : 'registerCommand';
    return `<div class="status warn">⚠ Not registered. The robot throws “Unknown ${type} ID” here. Add
      <code>DecisionTree.${fn}("${esc(ref)}", …)</code>.</div>`;
  }

  // ------------------------------------------------------------------ editing

  /**
   * Applies `mutator(root)` (which may mutate in place and/or return a new root) as one undo step.
   * Calls sharing a `coalesce` key with the previous step merge into it (used while typing).
   */
  function edit(mutator, { select, coalesce, keepPanel } = {}) {
    const cur = state.current;
    if (!cur) return;
    const top = state.undo[state.undo.length - 1];
    if (!coalesce || !top || top.coalesce !== coalesce) {
      state.undo.push({ root: structuredClone(cur.root), sel: state.selectedId, coalesce });
      if (state.undo.length > MAX_UNDO) state.undo.shift();
    }
    state.redo.length = 0;
    const result = mutator(cur.root);
    if (result) cur.root = result;
    if (select !== undefined) state.selectedId = select;
    render({ keepPanel });
  }

  function stepHistory(from, to) {
    const entry = from.pop();
    if (!entry || !state.current) return;
    to.push({ root: structuredClone(state.current.root), sel: state.selectedId });
    state.current.root = entry.root;
    state.selectedId = entry.sel;
    state.editSession++;
    render();
  }

  const selectedNode = () =>
    state.current && state.selectedId ? M.find(state.current.root, state.selectedId) : null;

  /** Swaps the subtree at `id` for `replacement` and selects it. */
  function replaceNode(id, replacement) {
    edit((root) => M.replace(root, id, replacement), { select: replacement.id });
  }

  /** Palette item dropped/clicked onto a node. */
  function applyItem(id, kind, ref) {
    const hit = M.find(state.current.root, id);
    if (!hit) return;
    const node = hit.node;
    if (kind === 'command') {
      if (node.type === 'command') return edit(() => void (node.ref = ref));
      replaceNode(id, M.leaf(ref));
      toast('Replaced the whole branch with a command (Ctrl+Z to undo)', 'warn');
    } else if (node.type === 'condition') {
      edit(() => void (node.ref = ref));
    } else {
      wrap(id, ref);
    }
  }

  /** Puts the node under a new condition as its yes branch. */
  function wrap(id, ref = '') {
    replaceNode(id, M.condition(ref, M.find(state.current.root, id).node, M.leaf('none')));
  }

  function setType(type) {
    const hit = selectedNode();
    if (!hit || hit.node.type === type) return;
    if (type === 'command') return replaceNode(hit.node.id, M.leaf('none'));
    wrap(hit.node.id);
    focusRefInput();
  }

  function deleteSelected() {
    const hit = selectedNode();
    if (hit) replaceNode(hit.node.id, M.leaf('none'));
  }

  function swapSelected() {
    const hit = selectedNode();
    if (!hit || hit.node.type !== 'condition') return;
    const n = hit.node;
    edit(() => void ([n.yes, n.no] = [n.no, n.yes]));
  }

  function copySelected() {
    const hit = selectedNode();
    if (!hit) return;
    state.clipboard = M.toRobotJson(hit.node);
    navigator.clipboard?.writeText(JSON.stringify(state.clipboard, null, 2)).catch(() => {});
    toast('Copied branch');
  }

  function pasteOnSelected() {
    const hit = selectedNode();
    if (hit && state.clipboard) replaceNode(hit.node.id, M.fromRobotJson(state.clipboard));
  }

  /** Arrow-key navigation: up = parent, left = yes branch, right = no branch. */
  function navigate(key) {
    const hit = selectedNode();
    if (!hit) {
      if (state.current) select(state.current.root.id);
      return;
    }
    const n = hit.node;
    const target =
      key === 'ArrowUp' ? hit.parent :
      n.type !== 'condition' ? null :
      key === 'ArrowRight' ? n.no : n.yes;
    if (target) select(target.id, true);
  }

  function select(id, reveal = false) {
    state.selectedId = id;
    if (state.tab !== 'inspect' && state.tab !== 'preview') state.tab = 'inspect';
    render();
    if (reveal) revealNode(id);
  }

  // ------------------------------------------------------------------ files

  async function setProject(project) {
    state.project = project;
    state.current = null;
    state.selectedId = null;
    state.undo = [];
    state.redo = [];
    await refreshProject();
    if (state.files.length) await openFile(state.files[0]);
    else render();
    toast(`Opened ${project.name}`);
  }

  async function refreshProject() {
    if (!state.project) return;
    [state.files, state.registry] = await Promise.all([state.project.list(), state.project.scanRegistry()]);
  }

  async function openFile(name) {
    const text = await state.project.read(name);
    let root;
    try {
      root = M.parse(text);
    } catch (e) {
      throw new Error(`${name}.json is not a valid strategy: ${e.message}`);
    }
    state.current = { name, root, saved: M.serialize(root) };
    state.selectedId = null;
    state.undo = [];
    state.redo = [];
    state.preview.clear();
    render();
    fit();
  }

  async function save() {
    const cur = state.current;
    if (!cur) return;
    const text = M.serialize(cur.root);
    await state.project.write(cur.name, text);
    cur.saved = text;
    if (!state.files.includes(cur.name)) state.files = await state.project.list();
    render();
    const n = state.issues.size;
    toast(n ? `Saved ${cur.name}.json with ${n} issue${n > 1 ? 's' : ''}` : `Saved ${cur.name}.json`, n ? 'warn' : '');
  }

  /** Throws if another strategy already has this name. Case-insensitive: Windows paths are. */
  async function assertNameFree(name, except = null) {
    const lower = name.toLowerCase();
    const taken = (await state.project.list()).some((f) => f !== except && f.toLowerCase() === lower);
    if (taken) throw new Error(`"${name}" already exists`);
  }

  async function createFile(name, root) {
    await assertNameFree(name);
    await state.project.write(name, M.serialize(root));
    state.files = await state.project.list();
    await openFile(name);
  }

  const fileActions = {
    async 'new-file'() {
      if (!state.project || !confirmDiscard()) return;
      const name = askName('Name of the new strategy:', 'New Strategy');
      if (name) await createFile(name, M.leaf('none'));
    },
    async 'duplicate-file'() {
      const cur = state.current;
      if (!cur) return;
      const name = askName(`Duplicate "${cur.name}" as:`, cur.name + ' Copy');
      if (name) await createFile(name, M.clone(cur.root)); // includes unsaved edits
    },
    async 'rename-file'() {
      const cur = state.current;
      if (!cur) return;
      const name = askName(`Rename "${cur.name}" to:`, cur.name);
      if (!name || name === cur.name) return;
      await assertNameFree(name, cur.name);
      // Moves the on-disk version; unsaved edits stay unsaved. For a case-only rename the old
      // and new names are the same file on Windows, so it must be removed before writing.
      if (name.toLowerCase() === cur.name.toLowerCase()) {
        await state.project.remove(cur.name);
        await state.project.write(name, cur.saved);
      } else {
        await state.project.write(name, cur.saved);
        await state.project.remove(cur.name);
      }
      cur.name = name;
      state.files = await state.project.list();
      render();
    },
    async 'delete-file'() {
      const cur = state.current;
      if (!cur || !confirm(`Delete "${cur.name}.json"? This cannot be undone.`)) return;
      await state.project.remove(cur.name);
      state.current = null;
      state.files = await state.project.list();
      render();
      toast(`Deleted ${cur.name}.json`);
    },
  };

  // ------------------------------------------------------------------ canvas view

  function applyView() {
    const { x, y, k } = state.view;
    $('world').style.transform = `translate(${x}px, ${y}px) scale(${k})`;
    $('zoom-readout').textContent = state.current ? Math.round(k * 100) + '%' : '';
  }

  const MIN_ZOOM = 0.15;
  const MAX_ZOOM = 2.5;
  let fitPending = false;

  function fit() {
    if (!state.current) return;
    const box = $('canvas').getBoundingClientRect();
    const pad = 60;
    // Not laid out yet (hidden tab, first paint): retry from the ResizeObserver.
    fitPending = box.width <= pad * 2 || box.height <= pad * 2;
    if (fitPending) return;
    const { width, height } = M.layout(state.current.root);
    const k = Math.max(MIN_ZOOM, Math.min(1.2, (box.width - pad * 2) / width, (box.height - pad * 2) / height));
    state.view = { k, x: (box.width - width * k) / 2, y: Math.max(pad, (box.height - height * k) / 2) };
    applyView();
  }

  function revealNode(id) {
    const p = M.layout(state.current.root).pos.get(id);
    const box = $('canvas').getBoundingClientRect();
    const v = state.view;
    const sx = v.x + p.x * v.k;
    const sy = v.y + p.y * v.k;
    const w = M.NODE_W * v.k;
    const h = M.NODE_H * v.k;
    const margin = 40;
    if (sx < margin) v.x += margin - sx;
    else if (sx + w > box.width - margin) v.x -= sx + w - (box.width - margin);
    if (sy < margin) v.y += margin - sy;
    else if (sy + h > box.height - margin) v.y -= sy + h - (box.height - margin);
    applyView();
  }

  function setupCanvas() {
    const canvas = $('canvas');
    let pan = null;

    canvas.addEventListener('pointerdown', (e) => {
      const node = e.target.closest('.node');
      if (node) {
        select(node.dataset.id);
        return;
      }
      if (e.target.closest('.welcome') || e.button !== 0) return;
      pan = { sx: e.clientX, sy: e.clientY, x: state.view.x, y: state.view.y, moved: false };
      canvas.setPointerCapture(e.pointerId);
      canvas.classList.add('panning');
    });
    canvas.addEventListener('pointermove', (e) => {
      if (!pan) return;
      state.view.x = pan.x + e.clientX - pan.sx;
      state.view.y = pan.y + e.clientY - pan.sy;
      pan.moved ||= Math.abs(e.clientX - pan.sx) + Math.abs(e.clientY - pan.sy) > 3;
      applyView();
    });
    canvas.addEventListener('pointerup', () => {
      if (pan && !pan.moved && state.selectedId) {
        state.selectedId = null; // plain click on empty canvas clears the selection
        render();
      }
      pan = null;
      canvas.classList.remove('panning');
    });

    canvas.addEventListener(
      'wheel',
      (e) => {
        e.preventDefault();
        const box = canvas.getBoundingClientRect();
        const mx = e.clientX - box.left;
        const my = e.clientY - box.top;
        const v = state.view;
        const k = Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, v.k * Math.exp(-e.deltaY * 0.0015)));
        v.x = mx - (mx - v.x) * (k / v.k);
        v.y = my - (my - v.y) * (k / v.k);
        v.k = k;
        applyView();
      },
      { passive: false }
    );

    // Drag & drop from the palette onto nodes.
    const dropTarget = (e) => (e.dataTransfer.types.includes(DRAG_MIME) ? e.target.closest('.node') : null);
    const highlight = (target) =>
      canvas.querySelectorAll('.node').forEach((el) => el.classList.toggle('drop-target', el === target));
    canvas.addEventListener('dragover', (e) => {
      const node = dropTarget(e);
      highlight(node);
      if (!node) return;
      e.preventDefault();
      e.dataTransfer.dropEffect = 'copy';
    });
    canvas.addEventListener('dragleave', (e) => {
      if (!canvas.contains(e.relatedTarget)) highlight(null);
    });
    canvas.addEventListener('drop', (e) => {
      const node = dropTarget(e);
      if (!node) return;
      e.preventDefault();
      const { kind, ref } = JSON.parse(e.dataTransfer.getData(DRAG_MIME));
      applyItem(node.dataset.id, kind, ref);
    });

    new ResizeObserver(() => fitPending && fit()).observe(canvas);
  }

  // ------------------------------------------------------------------ rendering

  function render({ keepPanel = false } = {}) {
    state.issues = state.current ? M.validate(state.current.root, state.registry) : new Map();
    state.dirty = isDirty();
    renderToolbar();
    renderSidebar();
    renderCanvas();
    if (!keepPanel) renderPanel();
  }

  function renderToolbar() {
    const cur = state.current;
    $('doc-title').innerHTML = cur ? esc(cur.name) + (state.dirty ? '<span class="dirty" title="Unsaved changes">●</span>' : '') : '';
    document.title = cur ? `${state.dirty ? '● ' : ''}${cur.name} – Strategy Editor` : 'Strategy Editor';
    const badge = $('issue-badge');
    badge.hidden = !state.issues.size;
    badge.textContent = `⚠ ${state.issues.size} issue${state.issues.size === 1 ? '' : 's'}`;
    const q = (a) => document.querySelector(`.toolbar [data-action="${a}"]`);
    q('undo').disabled = !state.undo.length;
    q('redo').disabled = !state.redo.length;
    q('fit').disabled = !cur;
    q('save').disabled = !cur;
  }

  function renderSidebar() {
    const p = state.project;
    $('project-name').innerHTML = p
      ? `<b>${esc(p.name)}</b>` + (p.isMemory ? '' : `<div class="hint">src/main/deploy/strategies</div>`)
      : '<span class="muted">No project open</span>';
    document.querySelector('[data-action="rescan"]').disabled = !p;
    $('import-btn').hidden = !p?.isMemory;

    const cur = state.current;
    $('file-list').innerHTML = !p
      ? ''
      : state.files.length
        ? state.files
            .map((f) => {
              const active = cur && cur.name === f;
              return `<li data-file="${esc(f)}" class="${active ? 'active' : ''}"><span>${esc(f)}</span>${active && state.dirty ? '<span class="dirty">●</span>' : ''}</li>`;
            })
            .join('')
        : '<li class="empty">No strategies yet. Click “+ New”.</li>';
    document.querySelector('[data-action="new-file"]').disabled = !p;
    for (const a of ['duplicate-file', 'rename-file', 'delete-file']) {
      document.querySelector(`[data-action="${a}"]`).disabled = !cur;
    }

    const chips = (type) =>
      knownIds(type)
        .map((id) => {
          const where = registrationOf(type, id);
          const cls = where === null ? ' unregistered' : '';
          const title = where ? `Registered in ${where}` : where === null ? 'Not registered in the robot code' : '';
          return `<span class="chip ${type}${cls}" draggable="true" data-kind="${type}" data-ref="${esc(id)}" title="${esc(title)}">${esc(id)}</span>`;
        })
        .join('');
    const howTo = '<code>DecisionTree.registerCondition("id", supplier)</code>';
    $('palette-commands').innerHTML = chips('command') || '<span class="hint">None found.</span>';
    $('palette-conditions').innerHTML =
      chips('condition') ||
      `<span class="hint">${
        state.registry
          ? `None registered yet. Add ${howTo} in the robot code, then Rescan.`
          : `Open the repo root to detect them, or type IDs in the Inspector. Register with ${howTo}.`
      }</span>`;
  }

  function renderCanvas() {
    const world = $('world');
    const overlay = $('overlay');
    const cur = state.current;
    overlay.innerHTML = cur ? '' : welcomeHtml();
    if (!cur) {
      world.innerHTML = '';
      applyView();
      return;
    }

    const W = M.NODE_W;
    const H = M.NODE_H;
    const { pos, width, height } = M.layout(cur.root);
    const path = state.tab === 'preview' ? M.trace(cur.root, (ref) => !!state.preview.get(ref)) : null;
    const onPath = new Set(path);
    const resultId = path && path[path.length - 1];

    let edges = '';
    let labels = '';
    let nodes = '';
    M.walk(cur.root, (n, parent, side) => {
      const p = pos.get(n.id);
      const dim = path && !onPath.has(n.id) ? ' dim' : '';
      if (parent) {
        const q = pos.get(parent.id);
        const x1 = q.x + W / 2;
        const y1 = q.y + H;
        const x2 = p.x + W / 2;
        const y2 = p.y;
        const my = (y1 + y2) / 2;
        const active = path && onPath.has(n.id) ? ' active' : '';
        edges += `<path class="edge ${side}${active}${dim}" d="M${x1} ${y1} C${x1} ${my} ${x2} ${my} ${x2} ${y2}"/>`;
        labels += `<div class="edge-label ${side}${dim}" style="left:${(x1 + x2) / 2}px;top:${my}px">${side}</div>`;
      }
      const issue = state.issues.get(n.id);
      const cls = [
        'node',
        n.type,
        n.id === state.selectedId && 'selected',
        issue && 'invalid',
        n.id === resultId && 'result',
      ].filter(Boolean).join(' ');
      nodes +=
        `<div class="${cls}${dim}" data-id="${n.id}" style="left:${p.x}px;top:${p.y}px;width:${W}px;height:${H}px" title="${esc(issue || n.ref)}">` +
        `<span class="node-kind">${n.type === 'condition' ? 'IF' : 'RUN'}</span>` +
        `<span class="node-ref">${n.ref ? esc(n.ref) : '<em>empty</em>'}</span>` +
        (issue ? '<span class="node-warn">!</span>' : '') +
        '</div>';
    });
    world.innerHTML = `<svg class="edges" width="${width}" height="${height}">${edges}</svg>${labels}${nodes}`;
    applyView();
  }

  function welcomeHtml() {
    if (state.project) {
      return `<div class="welcome"><h1>No strategy open</h1><p class="muted">Pick a strategy on the left or create a new one.</p>
        <div class="btn-row"><button class="primary" data-action="new-file">+ New strategy</button></div></div>`;
    }
    const reopen = state.lastFolder
      ? `<button class="primary" data-action="reopen">Reopen “${esc(state.lastFolder.name)}”</button>`
      : '';
    const body = P.supported
      ? `<p class="muted">Open the robot repo folder (the one with <code>build.gradle</code>). Strategies are saved to
         <code>src/main/deploy/strategies</code>, and command/condition IDs are read from the Java code.</p>
         <div class="btn-row">${reopen}<button class="${reopen ? '' : 'primary'}" data-action="open-folder">Open robot project…</button></div>`
      : `<p class="muted">This browser can’t open folders directly (use Chrome or Edge for that). You can still
         import strategy files and download your edits.</p>
         <div class="btn-row"><button class="primary" data-action="open-folder">Start without a folder</button></div>`;
    return `<div class="welcome"><h1>Bunnybots strategy editor</h1>${body}</div>`;
  }

  function renderPanel() {
    document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === state.tab));
    const body = $('panel-body');
    if (!state.current) {
      body.innerHTML = '<p class="muted">No strategy open.</p>';
      return;
    }
    body.innerHTML = state.tab === 'preview' ? previewHtml() : state.tab === 'json' ? jsonHtml() : inspectorHtml();
  }

  function inspectorHtml() {
    const hit = selectedNode();
    if (!hit) {
      const issues = [...state.issues].map(([id, msg]) => `<li data-select="${id}">${esc(msg)}</li>`).join('');
      return `<p class="muted">Select a node to edit it. Drag commands and conditions from the left onto nodes:
        a command replaces the node, a condition dropped on a command wraps it in an “if”.</p>
        <p class="hint">Keys: arrows to move around · Del to clear a branch · Ctrl+C / Ctrl+V to copy a branch · F to fit.</p>
        ${issues ? `<h2>Issues</h2><ul class="issues">${issues}</ul>` : '<p class="status ok">✓ No issues: every ID is registered.</p>'}`;
    }

    const n = hit.node;
    const isCond = n.type === 'condition';

    const crumbs = M.pathTo(state.current.root, n.id)
      .map(({ node, side }, i, all) => {
        const step = side ? `<span class="${side}">${side}</span> → ` : '';
        const label = i === all.length - 1 ? '<b>this node</b>' : `IF ${esc(node.ref || '?')}`;
        return step + label;
      })
      .join(' → ');

    const options = knownIds(n.type).map((id) => `<option value="${esc(id)}"></option>`).join('');
    return `
      <div class="field"><label>Node type</label>
        <div class="segmented">
          <button data-action="type-condition" class="${isCond ? 'on' : ''}">Condition (if)</button>
          <button data-action="type-command" class="${isCond ? '' : 'on'}">Command (run)</button>
        </div></div>
      <div class="field"><label for="ref-input">${isCond ? 'Condition ID' : 'Command ID'}</label>
        <input type="text" id="ref-input" list="ref-options" value="${esc(n.ref)}" spellcheck="false" autocomplete="off" />
        <datalist id="ref-options">${options}</datalist>
        <div id="ref-status">${refStatusHtml(n.type, n.ref)}</div></div>
      <div class="field"><label>Reached by</label><div class="crumbs">${crumbs}</div></div>
      <div class="field"><label>Actions</label>
        <div class="btn-row">
          ${isCond ? '<button class="small" data-action="swap">Swap yes/no</button>' : ''}
          <button class="small" data-action="wrap">Wrap in condition</button>
          ${hit.parent ? '<button class="small" data-action="select-parent">Select parent</button>' : ''}
          <button class="small" data-action="copy">Copy branch</button>
          <button class="small" data-action="paste" ${state.clipboard ? '' : 'disabled'}>Paste here</button>
          <button class="small danger" data-action="delete">${isCond ? 'Delete branch' : 'Reset to none'}</button>
        </div></div>`;
  }

  function previewHtml() {
    const root = state.current.root;
    const conds = M.usedRefs(root, 'condition');
    const path = M.trace(root, (ref) => !!state.preview.get(ref));
    const result = M.find(root, path[path.length - 1]).node;
    const toggles = conds
      .map((ref) => {
        const on = !!state.preview.get(ref);
        return `<label class="toggle"><input type="checkbox" data-cond="${esc(ref)}" ${on ? 'checked' : ''} />
          <span>${esc(ref)}</span><span class="val" style="color:var(--${on ? 'yes' : 'no'})">${on ? 'TRUE' : 'FALSE'}</span></label>`;
      })
      .join('');
    return `<p class="muted">Set what the robot “sees” and the highlighted path shows what
        <code>DecisionTree.evaluate()</code> picks. The robot re-evaluates every time a command finishes.</p>
      <div class="result-box">Robot would run<br /><b>${result.ref ? esc(result.ref) : '(empty)'}</b></div>
      ${conds.length ? `<h2>Conditions</h2>${toggles}
        <div class="btn-row" style="margin-top:10px"><button class="small" data-action="preview-reset">All false</button></div>`
        : '<p class="muted">This strategy has no conditions, so it always runs the same command.</p>'}`;
  }

  function jsonHtml() {
    return `<p class="muted">Exactly what gets written to <code>${esc(state.current.name)}.json</code>.</p>
      <pre class="json">${esc(M.serialize(state.current.root))}</pre>
      <button class="small" data-action="copy-json">Copy JSON</button>`;
  }

  function focusRefInput() {
    requestAnimationFrame(() => $('ref-input')?.focus());
  }

  // ------------------------------------------------------------------ events

  const actions = {
    undo: () => stepHistory(state.undo, state.redo),
    redo: () => stepHistory(state.redo, state.undo),
    fit,
    'next-issue': () => {
      const first = state.issues.keys().next().value;
      if (first) select(first, true);
    },
    save: () => run(save),
    rescan: () =>
      run(async () => {
        await refreshProject();
        render();
        toast('Rescanned project');
      }),
    'open-folder': () =>
      run(async () => {
        if (!confirmDiscard()) return;
        await setProject(P.supported ? await P.openWithPicker() : new P.MemoryProject());
      }),
    reopen: () =>
      run(async () => {
        if (state.lastFolder) await setProject(await P.reopen(state.lastFolder));
      }),
    import: () => $('import-input').click(),
    'type-condition': () => setType('condition'),
    'type-command': () => setType('command'),
    swap: swapSelected,
    wrap: () => {
      const hit = selectedNode();
      if (hit) {
        wrap(hit.node.id);
        focusRefInput();
      }
    },
    'select-parent': () => navigate('ArrowUp'),
    copy: copySelected,
    paste: pasteOnSelected,
    delete: deleteSelected,
    'preview-reset': () => {
      state.preview.clear();
      render();
    },
    'copy-json': () =>
      navigator.clipboard.writeText(M.serialize(state.current.root)).then(() => toast('Copied JSON')),
  };
  for (const [name, fn] of Object.entries(fileActions)) actions[name] = () => run(fn);

  function setupEvents() {
    document.addEventListener('click', (e) => {
      const action = e.target.closest('[data-action]');
      if (action && !action.disabled) return actions[action.dataset.action]?.();

      const file = e.target.closest('[data-file]');
      if (file) {
        const name = file.dataset.file;
        if (state.current?.name !== name && confirmDiscard()) run(() => openFile(name));
        return;
      }
      const tab = e.target.closest('[data-tab]');
      if (tab) {
        state.tab = tab.dataset.tab;
        return render();
      }
      const issue = e.target.closest('[data-select]');
      if (issue) return select(issue.dataset.select, true);

      const chip = e.target.closest('.chip');
      if (chip) {
        if (!selectedNode()) return toast('Select a node first, or drag the item onto a node');
        applyItem(state.selectedId, chip.dataset.kind, chip.dataset.ref);
      }
    });

    document.addEventListener('dragstart', (e) => {
      const chip = e.target.closest?.('.chip');
      if (!chip) return;
      e.dataTransfer.setData(DRAG_MIME, JSON.stringify({ kind: chip.dataset.kind, ref: chip.dataset.ref }));
      e.dataTransfer.effectAllowed = 'copy';
    });

    // Typing an ID updates the tree live; one focus of the box = one undo step.
    const panel = $('panel-body');
    panel.addEventListener('focusin', (e) => {
      if (e.target.id === 'ref-input') state.editSession++;
    });
    panel.addEventListener('input', (e) => {
      if (e.target.id === 'ref-input') {
        const hit = selectedNode();
        if (!hit) return;
        const value = e.target.value.trim();
        edit(() => void (hit.node.ref = value), { coalesce: `ref:${hit.node.id}:${state.editSession}`, keepPanel: true });
        $('ref-status').innerHTML = refStatusHtml(hit.node.type, value);
      } else if (e.target.dataset.cond) {
        state.preview.set(e.target.dataset.cond, e.target.checked);
        render();
      }
    });
    panel.addEventListener('keydown', (e) => {
      if (e.target.id === 'ref-input' && (e.key === 'Enter' || e.key === 'Escape')) {
        e.target.blur();
        renderPanel(); // refresh crumbs/status with the committed value
      }
    });

    $('import-input').addEventListener('change', (e) =>
      run(async () => {
        for (const file of e.target.files) {
          const name = file.name.replace(/\.json$/i, '');
          state.project.files.set(name, await file.text());
        }
        e.target.value = '';
        state.files = await state.project.list();
        if (!state.current && state.files.length) await openFile(state.files[0]);
        else render();
      })
    );

    document.addEventListener('keydown', (e) => {
      const mod = e.ctrlKey || e.metaKey;
      const key = e.key.toLowerCase();
      if (mod && key === 's') {
        e.preventDefault();
        return actions.save();
      }
      if (e.target.closest?.('input, textarea')) return;
      if (mod && key === 'z') {
        e.preventDefault();
        return e.shiftKey ? actions.redo() : actions.undo();
      }
      if (mod && key === 'y') {
        e.preventDefault();
        return actions.redo();
      }
      if (mod && key === 'c') return copySelected();
      if (mod && key === 'v') return pasteOnSelected();
      if (e.key === 'Delete' || e.key === 'Backspace') return deleteSelected();
      if (e.key.startsWith('Arrow')) {
        e.preventDefault();
        return navigate(e.key);
      }
      if (e.key === 'Escape') {
        state.selectedId = null;
        return render();
      }
      if (key === 'f' && !mod) return fit();
    });

    window.addEventListener('beforeunload', (e) => {
      if (isDirty()) e.preventDefault();
    });

    // Pick up strategies/IDs changed outside the editor (e.g. new registerCommand calls).
    window.addEventListener('focus', () => {
      if (state.project && !state.project.isMemory) run(() => refreshProject().then(() => render()));
    });
  }

  // ------------------------------------------------------------------ boot

  setupCanvas();
  setupEvents();
  render();
  P.lastFolder().then((handle) => {
    state.lastFolder = handle;
    if (!state.project) render();
  });

  // Exposed for debugging from the devtools console.
  window.strategyEditor = { state, setProject, openFile, render };
})();
