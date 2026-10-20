/*
 * Project access for the strategy editor.
 *
 * FsProject reads/writes the robot repo directly through the File System Access API
 * (Chrome/Edge). Strategies live in src/main/deploy/strategies/*.json, which is where
 * frc.robot.util.DecisionTree loads them from. Command/condition IDs are discovered by
 * scanning src/main/java for DecisionTree.registerCommand/registerCondition("id", ...).
 *
 * MemoryProject is the fallback for browsers without that API: files are imported by hand
 * and every save downloads the JSON.
 */
(function (global) {
  'use strict';

  const STRATEGY_DIR = 'strategies';
  const REGISTER_RE = /DecisionTree\s*\.\s*register(Command|Condition)\s*\(\s*"([^"]+)"/g;

  const emptyRegistry = () => ({
    // "none" is registered in DecisionTree's static initializer.
    commands: new Map([['none', 'DecisionTree.java (built-in)']]),
    conditions: new Map(),
  });

  async function getDir(parent, parts, create = false) {
    let dir = parent;
    for (const part of parts) dir = await dir.getDirectoryHandle(part, { create });
    return dir;
  }

  async function tryGetDir(parent, parts) {
    try {
      return await getDir(parent, parts);
    } catch (e) {
      if (e.name === 'NotFoundError' || e.name === 'TypeMismatchError') return null;
      throw e;
    }
  }

  const byName = (a, b) => a.localeCompare(b, undefined, { numeric: true, sensitivity: 'base' });

  class FsProject {
    constructor(root, deploy, java) {
      this.root = root;
      this.deploy = deploy;
      this.java = java;
      this.name = root.name;
    }

    /** Accepts the repo root (preferred) or the deploy folder itself. */
    static async fromHandle(root) {
      const deploy = await tryGetDir(root, ['src', 'main', 'deploy']);
      if (deploy) return new FsProject(root, deploy, await tryGetDir(root, ['src', 'main', 'java']));
      if (root.name === 'deploy') return new FsProject(root, root, null);
      throw new Error(`"${root.name}" has no src/main/deploy folder. Open the robot repo root.`);
    }

    strategyDir(create) {
      return create ? getDir(this.deploy, [STRATEGY_DIR], true) : tryGetDir(this.deploy, [STRATEGY_DIR]);
    }

    async list() {
      const dir = await this.strategyDir(false);
      if (!dir) return [];
      const names = [];
      for await (const [name, handle] of dir.entries()) {
        if (handle.kind === 'file' && name.endsWith('.json')) names.push(name.slice(0, -5));
      }
      return names.sort(byName);
    }

    async read(name) {
      const dir = await this.strategyDir(false);
      const file = await (await dir.getFileHandle(name + '.json')).getFile();
      return file.text();
    }

    async write(name, text) {
      const dir = await this.strategyDir(true);
      const writable = await (await dir.getFileHandle(name + '.json', { create: true })).createWritable();
      await writable.write(text);
      await writable.close();
    }

    async remove(name) {
      await (await this.strategyDir(false)).removeEntry(name + '.json');
    }

    /** Returns { commands, conditions } as Map<id, fileName>, or null if there is no Java source. */
    async scanRegistry() {
      if (!this.java) return null;
      const files = [];
      const collect = async (dir) => {
        const pending = [];
        for await (const [name, handle] of dir.entries()) {
          if (handle.kind === 'directory') pending.push(collect(handle));
          else if (name.endsWith('.java')) files.push(handle);
        }
        await Promise.all(pending);
      };
      await collect(this.java);

      const registry = emptyRegistry();
      await Promise.all(
        files.map(async (handle) => {
          const text = await (await handle.getFile()).text();
          for (const [, kind, id] of text.matchAll(REGISTER_RE)) {
            (kind === 'Command' ? registry.commands : registry.conditions).set(id, handle.name);
          }
        })
      );
      return registry;
    }
  }

  class MemoryProject {
    constructor() {
      this.name = 'Browser only (saves download)';
      this.isMemory = true;
      this.files = new Map();
    }
    async list() {
      return [...this.files.keys()].sort(byName);
    }
    async read(name) {
      return this.files.get(name);
    }
    async write(name, text) {
      this.files.set(name, text);
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([text], { type: 'application/json' }));
      a.download = name + '.json';
      a.click();
      URL.revokeObjectURL(a.href);
    }
    async remove(name) {
      this.files.delete(name);
    }
    async scanRegistry() {
      return null;
    }
  }

  /** Remembers the last opened folder handle in IndexedDB so it can be reopened in one click. */
  const LastFolder = {
    async _store(mode) {
      const db = await new Promise((resolve, reject) => {
        const req = indexedDB.open('strategy-editor', 1);
        req.onupgradeneeded = () => req.result.createObjectStore('handles');
        req.onsuccess = () => resolve(req.result);
        req.onerror = () => reject(req.error);
      });
      return db.transaction('handles', mode).objectStore('handles');
    },
    async get() {
      try {
        const store = await this._store('readonly');
        return await new Promise((resolve) => {
          const req = store.get('last');
          req.onsuccess = () => resolve(req.result || null);
          req.onerror = () => resolve(null);
        });
      } catch {
        return null;
      }
    },
    async set(handle) {
      try {
        (await this._store('readwrite')).put(handle, 'last');
      } catch {
        /* remembering the folder is a convenience only */
      }
    },
  };

  const supported = typeof global.showDirectoryPicker === 'function';

  async function openWithPicker() {
    const handle = await global.showDirectoryPicker({ id: 'bunnybots-robot-repo', mode: 'readwrite' });
    const project = await FsProject.fromHandle(handle);
    await LastFolder.set(handle);
    return project;
  }

  /** Must be called from a user gesture: re-granting permission shows a browser prompt. */
  async function reopen(handle) {
    if ((await handle.queryPermission({ mode: 'readwrite' })) !== 'granted' &&
        (await handle.requestPermission({ mode: 'readwrite' })) !== 'granted') {
      throw new Error('Permission to the folder was not granted');
    }
    return FsProject.fromHandle(handle);
  }

  global.StrategyProject = {
    supported,
    openWithPicker,
    reopen,
    lastFolder: () => LastFolder.get(),
    MemoryProject,
    emptyRegistry,
  };
})(this);
