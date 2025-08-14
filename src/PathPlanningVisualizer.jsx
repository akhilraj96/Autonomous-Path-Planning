import React, { useEffect, useMemo, useRef, useState } from "react";
import { motion, AnimatePresence } from "framer-motion";
import {
  Play,
  Pause,
  RotateCcw,
  Flag,
  MapPinned,
  Square,
  MousePointer2,
  Zap,
  Eraser,
  Settings,
  Github,
  Download,
  Upload,
  Grid as GridIcon,
  Shuffle,
  Info,
} from "lucide-react";

/********************
 * Autonomous Path Planning Visualizer
 * Single-file React app (drop into Next.js or Vite + Tailwind)
 * Features:
 * - Algorithms: A*, Dijkstra, (grid-based) RRT* approximation
 * - Draw walls & weighted cells, drag start/goal
 * - Animate visited nodes and final path
 * - Grid size + speed controls, diagonal toggle
 * - Random maze & random weights
 * - Export / Import scenarios (JSON)
 ********************/

/******************** Helpers ********************/
const DIRS4 = [
  [1, 0],
  [-1, 0],
  [0, 1],
  [0, -1],
];
const DIRS8 = [
  ...DIRS4,
  [1, 1],
  [1, -1],
  [-1, 1],
  [-1, -1],
];

const clamp = (n, lo, hi) => Math.max(lo, Math.min(hi, n));
const key = (r, c) => `${r},${c}`;
const manhattan = (a, b) => Math.abs(a.r - b.r) + Math.abs(a.c - b.c);
const euclid = (a, b) => Math.hypot(a.r - b.r, a.c - b.c);

function inBounds(r, c, rows, cols) {
  return r >= 0 && r < rows && c >= 0 && c < cols;
}

function neighbors(r, c, rows, cols, diag) {
  const dirs = diag ? DIRS8 : DIRS4;
  const out = [];
  for (const [dr, dc] of dirs) {
    const nr = r + dr,
      nc = c + dc;
    if (inBounds(nr, nc, rows, cols)) out.push({ r: nr, c: nc });
  }
  return out;
}

class MinHeap {
  constructor() {
    this.a = [];
  }
  push(x) {
    this.a.push(x);
    this.#up(this.a.length - 1);
  }
  pop() {
    if (!this.a.length) return null;
    const top = this.a[0];
    const last = this.a.pop();
    if (this.a.length) {
      this.a[0] = last;
      this.#down(0);
    }
    return top;
  }
  get size() {
    return this.a.length;
  }
  #up(i) {
    const a = this.a;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].prio <= a[i].prio) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  #down(i) {
    const a = this.a,
      n = a.length;
    while (true) {
      let l = i * 2 + 1,
        r = l + 1,
        m = i;
      if (l < n && a[l].prio < a[m].prio) m = l;
      if (r < n && a[r].prio < a[m].prio) m = r;
      if (m === i) break;
      [a[i], a[m]] = [a[m], a[i]];
      i = m;
    }
  }
}

function reconstruct(cameFrom, gkey) {
  const path = [];
  let cur = gkey;
  if (!cameFrom[cur]) return path;
  while (cur) {
    const [r, c] = cur.split(",").map(Number);
    path.push({ r, c });
    cur = cameFrom[cur];
  }
  return path.reverse();
}

/******************** Algorithms ********************/
function runDijkstra(grid, start, goal, diag) {
  const rows = grid.length,
    cols = grid[0].length;
  const pq = new MinHeap();
  const dist = {};
  const came = {};
  const visited = [];
  const sKey = key(start.r, start.c);
  dist[sKey] = 0;
  pq.push({ prio: 0, node: sKey });

  while (pq.size) {
    const cur = pq.pop();
    const [r, c] = cur.node.split(",").map(Number);
    if (cur.prio !== dist[cur.node]) continue; // stale
    visited.push({ r, c });
    if (r === goal.r && c === goal.c) break;

    for (const nb of neighbors(r, c, rows, cols, diag)) {
      if (grid[nb.r][nb.c].wall) continue;
      const w = grid[nb.r][nb.c].weight ?? 1;
      const nk = key(nb.r, nb.c);
      const alt = dist[cur.node] + w;
      if (dist[nk] === undefined || alt < dist[nk]) {
        dist[nk] = alt;
        came[nk] = cur.node;
        pq.push({ prio: alt, node: nk });
      }
    }
  }
  const gKey = key(goal.r, goal.c);
  return { visited, path: reconstruct(came, gKey) };
}

function runAStar(grid, start, goal, diag) {
  const rows = grid.length,
    cols = grid[0].length;
  const pq = new MinHeap();
  const g = {};
  const f = {};
  const came = {};
  const visited = [];
  const sKey = key(start.r, start.c);
  g[sKey] = 0;
  f[sKey] = (diag ? euclid : manhattan)(start, goal);
  pq.push({ prio: f[sKey], node: sKey });

  while (pq.size) {
    const cur = pq.pop();
    const [r, c] = cur.node.split(",").map(Number);
    if (cur.prio !== f[cur.node]) continue; // stale
    visited.push({ r, c });
    if (r === goal.r && c === goal.c) break;

    for (const nb of neighbors(r, c, rows, cols, diag)) {
      if (grid[nb.r][nb.c].wall) continue;
      const w = grid[nb.r][nb.c].weight ?? 1;
      const nk = key(nb.r, nb.c);
      const tentative = g[cur.node] + w;
      if (g[nk] === undefined || tentative < g[nk]) {
        g[nk] = tentative;
        f[nk] = tentative + (diag ? euclid : manhattan)(nb, goal);
        came[nk] = cur.node;
        pq.push({ prio: f[nk], node: nk });
      }
    }
  }
  const gKey = key(goal.r, goal.c);
  return { visited, path: reconstruct(came, gKey) };
}

// Lightweight grid-based RRT* approximation for demo purposes
function runRRTStar(grid, start, goal, opts = {}) {
  const rows = grid.length,
    cols = grid[0].length;
  const iterations = opts.iterations ?? 1500;
  const radius = opts.radius ?? 4;
  const step = opts.step ?? 2;
  const goalBias = opts.goalBias ?? 0.08; // probability to sample goal

  const nodes = [{ r: start.r, c: start.c, parent: -1, cost: 0 }];
  const visited = [{ r: start.r, c: start.c }];

  const idx = (r, c) => r * cols + c;
  const isFree = (r, c) => inBounds(r, c, rows, cols) && !grid[r][c].wall;
  const dist = (a, b) => Math.hypot(a.r - b.r, a.c - b.c);

  function lineFree(a, b) {
    // Bresenham-like discrete collision check along line
    let x0 = a.r,
      y0 = a.c,
      x1 = b.r,
      y1 = b.c;
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;
    while (true) {
      if (!isFree(x0, y0)) return false;
      if (x0 === x1 && y0 === y1) break;
      const e2 = err * 2;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
    return true;
  }

  function nearest(pt) {
    let best = 0,
      bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) {
      const d = dist(pt, nodes[i]);
      if (d < bestD) {
        best = i;
        bestD = d;
      }
    }
    return best;
  }

  function steer(from, to) {
    const d = dist(from, to);
    if (d <= step) return to;
    const t = step / d;
    return {
      r: Math.round(from.r + (to.r - from.r) * t),
      c: Math.round(from.c + (to.c - from.c) * t),
    };
  }

  let goalIndex = -1;

  for (let it = 0; it < iterations; it++) {
    const sample = Math.random() < goalBias
      ? { r: goal.r, c: goal.c }
      : { r: Math.floor(Math.random() * rows), c: Math.floor(Math.random() * cols) };
    if (!isFree(sample.r, sample.c)) continue;

    const ni = nearest(sample);
    const newPt = steer(nodes[ni], sample);
    if (!isFree(newPt.r, newPt.c)) continue;
    if (!lineFree(nodes[ni], newPt)) continue;

    // cost uses weight map to prefer low-cost corridors
    const w = grid[newPt.r][newPt.c].weight ?? 1;
    const newCost = nodes[ni].cost + w * dist(nodes[ni], newPt);

    const node = { ...newPt, parent: ni, cost: newCost };
    const newIndex = nodes.push(node) - 1;
    visited.push({ r: node.r, c: node.c });

    // rewire within radius
    for (let j = 0; j < nodes.length - 1; j++) {
      const nbh = nodes[j];
      if (dist(nbh, node) <= radius && lineFree(node, nbh)) {
        const wj = grid[nbh.r][nbh.c].weight ?? 1;
        const alt = node.cost + wj * dist(node, nbh);
        if (alt < nbh.cost) {
          nbh.cost = alt;
          nbh.parent = newIndex;
        }
      }
    }

    if (dist(node, goal) <= step && lineFree(node, goal)) {
      // connect to goal
      const gIdx = nodes.push({ r: goal.r, c: goal.c, parent: newIndex, cost: node.cost }) - 1;
      goalIndex = gIdx;
      break;
    }
  }

  const path = [];
  if (goalIndex !== -1) {
    let cur = goalIndex;
    while (cur !== -1) {
      const n = nodes[cur];
      path.push({ r: n.r, c: n.c });
      cur = n.parent;
    }
    path.reverse();
  }

  return { visited, path };
}

/******************** UI + State ********************/
const DEFAULT_ROWS = 24;
const DEFAULT_COLS = 38;

function makeGrid(rows, cols) {
  return Array.from({ length: rows }, () =>
    Array.from({ length: cols }, () => ({ wall: false, weight: 1 }))
  );
}

const Tools = {
  WALL: "wall",
  ERASE: "erase",
  WEIGHT: "weight",
  START: "start",
  GOAL: "goal",
};

const Algorithms = {
  ASTAR: "A*",
  DIJKSTRA: "Dijkstra",
  RRTSTAR: "RRT*",
};

export default function PathPlanningVisualizer() {
  const [rows, setRows] = useState(DEFAULT_ROWS);
  const [cols, setCols] = useState(DEFAULT_COLS);
  const [grid, setGrid] = useState(() => makeGrid(DEFAULT_ROWS, DEFAULT_COLS));
  const [start, setStart] = useState({ r: 2, c: 2 });
  const [goal, setGoal] = useState({ r: DEFAULT_ROWS - 3, c: DEFAULT_COLS - 3 });
  const [tool, setTool] = useState(Tools.WALL);
  const [algo, setAlgo] = useState(Algorithms.ASTAR);
  const [diag, setDiag] = useState(false);
  const [speed, setSpeed] = useState(28); // ms per step
  const [animating, setAnimating] = useState(false);
  const [visited, setVisited] = useState([]);
  const [path, setPath] = useState([]);
  const [mouseDown, setMouseDown] = useState(false);
  const [weightBrush, setWeightBrush] = useState(5);

  useEffect(() => {
    setGrid(makeGrid(rows, cols));
    setStart({ r: 2, c: 2 });
    setGoal({ r: rows - 3, c: cols - 3 });
    setVisited([]); setPath([]); setAnimating(false);
  }, [rows, cols]);

  function mutateCell(r, c, fn) {
    setGrid((g) => {
      const ng = g.map((row) => row.slice());
      ng[r][c] = { ...ng[r][c], ...fn(ng[r][c]) };
      return ng;
    });
  }

  function handleCell(r, c) {
    if (animating) return;
    if (tool === Tools.START) {
      setStart({ r, c });
      return;
    }
    if (tool === Tools.GOAL) {
      setGoal({ r, c });
      return;
    }
    if (tool === Tools.WALL) {
      mutateCell(r, c, (cell) => ({ wall: true, weight: 1 }));
    } else if (tool === Tools.ERASE) {
      mutateCell(r, c, () => ({ wall: false, weight: 1 }));
    } else if (tool === Tools.WEIGHT) {
      mutateCell(r, c, () => ({ wall: false, weight: clamp(weightBrush, 1, 20) }));
    }
  }

  function handleDrag(r, c) {
    if (!mouseDown) return;
    handleCell(r, c);
  }

  function runOnce() {
    if (animating) return;
    let result;
    if (algo === Algorithms.DIJKSTRA) result = runDijkstra(grid, start, goal, diag);
    else if (algo === Algorithms.RRTSTAR) result = runRRTStar(grid, start, goal, { iterations: 2500, radius: 4, step: 2 });
    else result = runAStar(grid, start, goal, diag);
    setVisited(result.visited);
    setPath(result.path);
  }

  async function animate() {
    if (animating) return;
    setVisited([]);
    setPath([]);
    setAnimating(true);
  
    let result;
    if (algo === Algorithms.DIJKSTRA) result = runDijkstra(grid, start, goal, diag);
    else if (algo === Algorithms.RRTSTAR) result = runRRTStar(grid, start, goal, { iterations: 2500, radius: 4, step: 2 });
    else result = runAStar(grid, start, goal, diag);
  
    // Animate visited in batches
    const batchSize = 3; // try 3–5 for smoother performance
    for (let i = 0; i < result.visited.length; i += batchSize) {
      setVisited((v) => [...v, ...result.visited.slice(i, i + batchSize)]);
      await new Promise((res) => setTimeout(res, speed));
    }
  
    // Animate path separately
    for (let i = 0; i < result.path.length; i++) {
      setPath((p) => [...p, result.path[i]]);
      await new Promise((res) => setTimeout(res, speed));
    }
  
    setAnimating(false);
  }

  function reset() {
    setVisited([]);
    setPath([]);
    setAnimating(false);
  }

  function clearAll() {
    setGrid(makeGrid(rows, cols));
    reset();
  }

  function exportJSON() {
    const data = { rows, cols, start, goal, grid };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "path_planner_scene.json";
    a.click();
    URL.revokeObjectURL(url);
  }

  function importJSON(e) {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        if (!data.grid || !data.rows || !data.cols) return;
        setRows(data.rows);
        setCols(data.cols);
        setGrid(data.grid);
        setStart(data.start);
        setGoal(data.goal);
        reset();
      } catch (err) {
        console.error("Invalid JSON", err);
      }
    };
    reader.readAsText(file);
  }

  function randomMaze(density = 0.28) {
    setGrid((g) =>
      g.map((row, r) =>
        row.map((cell, c) => {
          if ((r === start.r && c === start.c) || (r === goal.r && c === goal.c)) return { wall: false, weight: 1 };
          return Math.random() < density ? { wall: true, weight: 1 } : { wall: false, weight: 1 };
        })
      )
    );
    reset();
  }

  function randomWeights(prob = 0.25) {
    setGrid((g) =>
      g.map((row, r) =>
        row.map((cell, c) => {
          if (cell.wall) return { wall: true, weight: 1 };
          if (Math.random() < prob) return { wall: false, weight: 6 + Math.floor(Math.random() * 8) };
          return { wall: false, weight: 1 };
        })
      )
    );
    reset();
  }

  const visitedSet = useMemo(() => new Set(visited.map((v) => key(v.r, v.c))), [visited]);
  const pathSet = useMemo(() => new Set(path.map((p) => key(p.r, p.c))), [path]);

  return (
    <div className="min-h-screen w-full bg-slate-950 text-slate-100 px-4 py-6">
      <div className="max-w-7xl mx-auto">
        <header className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 mb-6">
          <div>
            <h1 className="text-2xl sm:text-3xl font-bold tracking-tight flex items-center gap-2">
              <MapPinned className="h-7 w-7" /> Autonomous Path Planning Visualizer
            </h1>
            <p className="text-slate-400 text-sm mt-1">
              Draw obstacles & weights, choose an algorithm, and watch the planner compute a route from Start to Goal.
            </p>
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={animate}
              disabled={animating}
              className="inline-flex items-center gap-2 rounded-2xl px-4 py-2 bg-emerald-500 hover:bg-emerald-400 active:scale-[.98] shadow-lg shadow-emerald-500/20 disabled:opacity-50"
            >
              <Play className="h-4 w-4" /> Animate
            </button>
            <button
              onClick={runOnce}
              className="inline-flex items-center gap-2 rounded-2xl px-3 py-2 bg-sky-500 hover:bg-sky-400 active:scale-[.98] shadow-lg shadow-sky-500/20"
            >
              <Zap className="h-4 w-4" /> Compute
            </button>
            <button
              onClick={reset}
              className="inline-flex items-center gap-2 rounded-2xl px-3 py-2 bg-slate-800 hover:bg-slate-700"
            >
              <RotateCcw className="h-4 w-4" /> Reset
            </button>
            <button
              onClick={clearAll}
              className="inline-flex items-center gap-2 rounded-2xl px-3 py-2 bg-slate-800 hover:bg-slate-700"
            >
              <Eraser className="h-4 w-4" /> Clear
            </button>
          </div>
        </header>

        {/* Controls */}
        <section className="grid md:grid-cols-3 gap-4 mb-6">
          <div className="rounded-2xl bg-slate-900/70 border border-slate-800 p-4">
            <h2 className="font-semibold mb-3 flex items-center gap-2"><Settings className="h-4 w-4"/> Planner</h2>
            <div className="grid grid-cols-2 gap-2 text-sm">
              {Object.values(Algorithms).map((a) => (
                <button
                  key={a}
                  onClick={() => setAlgo(a)}
                  className={`rounded-xl px-3 py-2 border ${
                    algo === a
                      ? "bg-white text-slate-900 border-white"
                      : "bg-slate-800 border-slate-700 hover:bg-slate-700"
                  }`}
                >
                  {a}
                </button>
              ))}
            </div>
            <div className="mt-4 flex items-center justify-between text-sm">
              <label className="flex items-center gap-2">
                <input type="checkbox" className="accent-emerald-500" checked={diag} onChange={(e) => setDiag(e.target.checked)} />
                Allow diagonals
              </label>
              <div className="flex items-center gap-2">
                <span>Speed</span>
                <input
                  type="range"
                  min={4}
                  max={100}
                  value={speed}
                  onChange={(e) => setSpeed(Number(e.target.value))}
                />
              </div>
            </div>
          </div>

          <div className="rounded-2xl bg-slate-900/70 border border-slate-800 p-4">
            <h2 className="font-semibold mb-3 flex items-center gap-2"><GridIcon className="h-4 w-4"/> Grid</h2>
            <div className="grid grid-cols-2 gap-2 text-sm">
              <label className="flex items-center justify-between gap-2 bg-slate-800 rounded-xl px-3 py-2">
                <span>Rows</span>
                <input
                  type="number"
                  min={8}
                  max={60}
                  value={rows}
                  onChange={(e) => setRows(clamp(parseInt(e.target.value || "0"), 8, 60))}
                  className="w-20 bg-transparent outline-none text-right"
                />
              </label>
              <label className="flex items-center justify-between gap-2 bg-slate-800 rounded-xl px-3 py-2">
                <span>Cols</span>
                <input
                  type="number"
                  min={8}
                  max={80}
                  value={cols}
                  onChange={(e) => setCols(clamp(parseInt(e.target.value || "0"), 8, 80))}
                  className="w-20 bg-transparent outline-none text-right"
                />
              </label>
            </div>
            <div className="mt-3 grid grid-cols-2 gap-2 text-sm">
              <button onClick={() => randomMaze(0.28)} className="rounded-xl px-3 py-2 bg-slate-800 hover:bg-slate-700">Random maze</button>
              <button onClick={() => randomWeights(0.3)} className="rounded-xl px-3 py-2 bg-slate-800 hover:bg-slate-700">Random weights</button>
            </div>
          </div>

          <div className="rounded-2xl bg-slate-900/70 border border-slate-800 p-4">
            <h2 className="font-semibold mb-3 flex items-center gap-2"><MousePointer2 className="h-4 w-4"/> Tools</h2>
            <div className="grid grid-cols-5 gap-2 text-xs">
              <ToolButton label="Start" active={tool===Tools.START} onClick={()=>setTool(Tools.START)} icon={<Flag className="h-4 w-4"/>}/>
              <ToolButton label="Goal" active={tool===Tools.GOAL} onClick={()=>setTool(Tools.GOAL)} icon={<MapPinned className="h-4 w-4"/>}/>
              <ToolButton label="Wall" active={tool===Tools.WALL} onClick={()=>setTool(Tools.WALL)} icon={<Square className="h-4 w-4"/>}/>
              <ToolButton label="Erase" active={tool===Tools.ERASE} onClick={()=>setTool(Tools.ERASE)} icon={<Eraser className="h-4 w-4"/>}/>
              <ToolButton label="Weight" active={tool===Tools.WEIGHT} onClick={()=>setTool(Tools.WEIGHT)} icon={<Zap className="h-4 w-4"/>}/>
            </div>
            {tool===Tools.WEIGHT && (
              <div className="mt-3 text-sm flex items-center justify-between gap-2">
                <span>Weight brush</span>
                <input type="range" min={2} max={20} value={weightBrush} onChange={(e)=>setWeightBrush(Number(e.target.value))}/>
                <span className="tabular-nums bg-slate-800 rounded px-2 py-1">{weightBrush}</span>
              </div>
            )}
            <div className="mt-3 text-xs text-slate-400">
              Tip: Click to paint. Hold & drag to draw. Use Start/Goal tools to place endpoints.
            </div>
          </div>
        </section>

        {/* Grid */}
        <section className="rounded-2xl bg-slate-900/60 border border-slate-800 p-3">
          <div
            className="select-none"
            onMouseLeave={() => setMouseDown(false)}
          >
            <div className="grid" style={{ gridTemplateColumns: `repeat(${cols}, minmax(0, 1fr))` }}>
              {grid.map((row, r) =>
                row.map((cell, c) => {
                  const k = key(r, c);
                  const isStart = r === start.r && c === start.c;
                  const isGoal = r === goal.r && c === goal.c;
                  const v = visitedSet.has(k);
                  const p = pathSet.has(k);
                  const base = cell.wall
                    ? "bg-slate-700"
                    : cell.weight > 1
                    ? "bg-amber-900/30"
                    : "bg-slate-900";
                  const overlay = p
                    ? "bg-emerald-400"
                    : v
                    ? "bg-sky-700/40"
                    : "";
                  return (
                    <div
                      key={k}
                      className={`aspect-square border border-slate-800 ${base} ${overlay} relative cursor-crosshair`}
                      onMouseDown={() => {
                        setMouseDown(true);
                        handleCell(r, c);
                      }}
                      onMouseUp={() => setMouseDown(false)}
                      onMouseEnter={() => handleDrag(r, c)}
                      onClick={() => handleCell(r, c)}
                      title={`r${r} c${c}${cell.wall?" wall":""}${cell.weight>1?` w=${cell.weight}`:""}`}
                    >
                      {isStart && (
                        <div className="absolute inset-0 flex items-center justify-center text-sky-500 text-[10px]">
                          <Flag className="h-4 w-4"/>
                        </div>
                      )}
                      {isGoal && (
                        <div className="absolute inset-0 flex items-center justify-center text-pink-400 text-[10px]">
                          <MapPinned className="h-4 w-4"/>
                        </div>
                      )}
                      {!cell.wall && cell.weight>1 && (
                        <span className="absolute bottom-0 right-0 text-[10px] text-amber-400/90 p-0.5">{cell.weight}</span>
                      )}
                    </div>
                  );
                })
              )}
            </div>
          </div>

          {/* Legend + Export */}
          <div className="flex flex-wrap items-center justify-between gap-3 mt-3 text-xs text-slate-400">
            <div className="flex items-center gap-3">
              <Legend swatch="bg-slate-700" label="Wall"/>
              <Legend swatch="bg-amber-900/50" label="Weighted"/>
              <Legend swatch="bg-sky-700/60" label="Visited"/>
              <Legend ring label="Path"/>
            </div>
            <div className="flex items-center gap-2">
              <button onClick={exportJSON} className="inline-flex items-center gap-1 rounded-xl px-3 py-1.5 bg-slate-800 hover:bg-slate-700"><Download className="h-3.5 w-3.5"/> Export</button>
              <label className="inline-flex items-center gap-1 rounded-xl px-3 py-1.5 bg-slate-800 hover:bg-slate-700 cursor-pointer">
                <Upload className="h-3.5 w-3.5"/> Import
                <input type="file" accept="application/json" onChange={importJSON} className="hidden"/>
              </label>
            </div>
          </div>
        </section>

        {/* Footer */}
        <footer className="mt-6 text-xs text-slate-500 flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Info className="h-3.5 w-3.5"/>
            <p>
              A* uses {diag?"Euclidean":"Manhattan"} heuristic. Dijkstra ignores the heuristic. RRT* is an approximate, discrete demo.
            </p>
          </div>
          <a href="#" className="hover:text-slate-300 inline-flex items-center gap-1">
            <Github className="h-3.5 w-3.5"/> GitHub
          </a>
        </footer>
      </div>
    </div>
  );
}

function ToolButton({ label, icon, active, onClick }) {
  return (
    <button onClick={onClick} className={`flex flex-col items-center justify-center gap-1 rounded-xl px-2 py-2 border ${active?"border-white bg-white text-slate-900":"border-slate-700 bg-slate-800 hover:bg-slate-700"}`}>
      <div className="h-5 w-5">{icon}</div>
      <span>{label}</span>
    </button>
  );
}

function Legend({ swatch, label, ring }) {
  return (
    <div className="flex items-center gap-1.5">
      <span className={`h-3 w-3 rounded-sm border border-slate-700 ${swatch || "bg-transparent"} ${ring?"ring-2 ring-emerald-400": ""}`}></span>
      <span>{label}</span>
    </div>
  );
}
