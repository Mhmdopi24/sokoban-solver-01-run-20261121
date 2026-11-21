// Sokoban solver — clean-room implementation, written for this competition.
//
// Strategy: a generated level is a one-wide hallway with 1-4 rooms off it,
// and a box pushed out of a room can never come back, so each room is an
// independent sub-problem and the level optimum is the sum of the room
// optima. Each room is solved push-optimally with A* over pushes
// (admissible minimum-assignment box-to-goal heuristic, dead-square and
// frozen-box pruning), then the room solutions are stitched together with
// shortest player walks through the hallway. The whole run is budgeted by
// fixed node-expansion counts, never by the clock, so two executions
// write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed work budgets (node expansions), never wall-clock time.
const ASTAR_BUDGET_PER_ROOM = 150000;
const GREEDY_BUDGET_PER_ROOM = 120000;
const GLOBAL_EXPANSION_CAP = 1000000;

let globalExpansions = 0;

const DY = [-1, 1, 0, 0];
const DX = [0, 0, -1, 1];
const STEP = ['u', 'd', 'l', 'r'];
const OPP = [1, 0, 3, 2];

interface Level {
  w: number;
  h: number;
  wall: boolean[];
  goals: boolean[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const lines = text.replace(/\r/g, '').split('\n');
  while (lines.length > 0 && lines[lines.length - 1] === '') lines.pop();
  const h = lines.length;
  const w = Math.max.apply(null, lines.map((l) => l.length));
  const wall = new Array<boolean>(w * h).fill(true);
  const goals = new Array<boolean>(w * h).fill(false);
  const boxes: number[] = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = x < lines[y].length ? lines[y][x] : '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = false;
      if (ch === '.' || ch === '*') goals[i] = true;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ---------------------------------------------------------------- rooms ---

interface Room {
  cells: number[]; // grid indices of the room floor, door included
  cellOf: Map<number, number>; // grid index -> cell index
  door: number; // grid index of the door cell
  doorIdx: number; // cell index of the door cell
  entry: number; // grid index of the hallway cell just outside the door
  entryIdx: number; // pseudo cell index == cells.length
  goals: number[]; // goal cell indices
  boxes: number[]; // initial box cell indices
  n: number; // cells.length
}

// Split a level into its rooms: the hallway is the row the player starts
// in; removing it disconnects the rooms from each other.
function findRooms(level: Level): Room[] {
  const { w, h, wall } = level;
  const hallRow = Math.floor(level.player / w);
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let s = 0; s < w * h; s++) {
    if (wall[s] || Math.floor(s / w) === hallRow || seen[s]) continue;
    const cells: number[] = [];
    const q: number[] = [s];
    seen[s] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      cells.push(c);
      const y = Math.floor(c / w);
      const x = c % w;
      for (let d = 0; d < 4; d++) {
        const ny = y + DY[d];
        const nx = x + DX[d];
        if (ny < 0 || ny >= h || nx < 0 || nx >= w) continue;
        const nc = ny * w + nx;
        if (wall[nc] || ny === hallRow || seen[nc]) continue;
        seen[nc] = 1;
        q.push(nc);
      }
    }
    const roomBoxes = level.boxes.filter((b) => cells.indexOf(b) >= 0);
    if (roomBoxes.length === 0) continue;
    // The door is the room's only cell touching the hallway row.
    const doors = cells
      .filter((c) => Math.abs(Math.floor(c / w) - hallRow) === 1)
      .sort((a, b) => a - b);
    const door = doors[0];
    const entry = hallRow * w + (door % w);
    const cellOf = new Map<number, number>();
    cells.forEach((c, i) => cellOf.set(c, i));
    const goals: number[] = [];
    const boxes: number[] = [];
    cells.forEach((c, i) => {
      if (level.goals[c]) goals.push(i);
      if (roomBoxes.indexOf(c) >= 0) boxes.push(i);
    });
    boxes.sort((a, b) => a - b);
    rooms.push({
      cells,
      cellOf,
      door,
      doorIdx: cellOf.get(door) as number,
      entry,
      entryIdx: cells.length,
      goals,
      boxes,
      n: cells.length,
    });
  }
  // Deterministic order: by door column.
  rooms.sort((a, b) => (a.door % w) - (b.door % w));
  return rooms;
}

// ------------------------------------------------------- per-room solver ---

// Neighbour table over room cells plus the entry pseudo-cell (index n):
// nbr[i][d] is the cell index in direction d, or -1 when outside the
// walkable domain (room cells + entry). A box may only ever occupy a
// room cell, never the entry.
class RoomSolver {
  room: Room;
  n: number; // room cells; entry pseudo-index is n
  cy: number[];
  cx: number[];
  nbr: number[][];
  goalSet: boolean[];
  distGoal: number[][]; // distGoal[g][cell]: box-only distance goal->cell
  dead: boolean[]; // dead[c]: no box on c can ever reach a goal

  constructor(room: Room, level: Level) {
    this.room = room;
    this.n = room.n;
    const n = room.n;
    this.cy = new Array<number>(n + 1);
    this.cx = new Array<number>(n + 1);
    for (let i = 0; i < n; i++) {
      this.cy[i] = Math.floor(room.cells[i] / level.w);
      this.cx[i] = room.cells[i] % level.w;
    }
    this.cy[n] = Math.floor(room.entry / level.w);
    this.cx[n] = room.entry % level.w;
    // Neighbour table.
    this.nbr = [];
    for (let i = 0; i <= n; i++) {
      const row: number[] = [-1, -1, -1, -1];
      for (let d = 0; d < 4; d++) {
        const ny = this.cy[i] + DY[d];
        const nx = this.cx[i] + DX[d];
        if (ny < 0 || ny >= level.h || nx < 0 || nx >= level.w) continue;
        const g = ny * level.w + nx;
        if (g === room.entry) row[d] = n;
        else {
          const ci = room.cellOf.get(g);
          if (ci !== undefined) row[d] = ci;
        }
      }
      this.nbr.push(row);
    }
    this.goalSet = new Array<boolean>(n).fill(false);
    for (const g of room.goals) this.goalSet[g] = true;
    // Box-only distances from each goal, over room cells.
    this.distGoal = [];
    for (const g of room.goals) {
      const dist = new Array<number>(n).fill(Infinity);
      dist[g] = 0;
      const q: number[] = [g];
      for (let qi = 0; qi < q.length; qi++) {
        const c = q[qi];
        for (let d = 0; d < 4; d++) {
          const nb = this.nbr[c][d];
          if (nb < 0 || nb >= n || dist[nb] !== Infinity) continue;
          dist[nb] = dist[c] + 1;
          q.push(nb);
        }
      }
      this.distGoal.push(dist);
    }
    // Dead squares: reverse reachability from the goals. From a cell c
    // a box can have come from predecessor p = c - d when the player
    // could stand at c - 2d (a room cell, or the hallway entry).
    const reach = new Array<boolean>(n).fill(false);
    const rq: number[] = [];
    for (const g of room.goals) {
      reach[g] = true;
      rq.push(g);
    }
    for (let qi = 0; qi < rq.length; qi++) {
      const c = rq[qi];
      for (let d = 0; d < 4; d++) {
        const p = this.nbr[c][OPP[d]];
        if (p < 0 || p >= n || reach[p]) continue;
        const sy = this.cy[c] - 2 * DY[d];
        const sx = this.cx[c] - 2 * DX[d];
        if (sy < 0 || sy >= level.h || sx < 0 || sx >= level.w) continue;
        const sg = sy * level.w + sx;
        if (sg !== room.entry && room.cellOf.get(sg) === undefined) continue;
        reach[p] = true;
        rq.push(p);
      }
    }
    this.dead = reach.map((r) => !r);
  }

  // Minimum-cost assignment of boxes to goals (brute force, <= 4 boxes).
  // Admissible: one push moves one box one cell, so the sum of the
  // assigned distances can drop by at most 1 per push.
  heuristic(boxes: number[]): number {
    const m = boxes.length;
    const used = new Array<boolean>(m).fill(false);
    let best = Infinity;
    const rec = (i: number, sum: number): void => {
      if (sum >= best) return;
      if (i === m) {
        best = sum;
        return;
      }
      for (let g = 0; g < m; g++) {
        if (used[g]) continue;
        const d = this.distGoal[g][boxes[i]];
        if (d === Infinity) continue;
        used[g] = true;
        rec(i + 1, sum + d);
        used[g] = false;
      }
    };
    rec(0, 0);
    return best;
  }

  // True when some box can never move again and is not on a goal.
  // Fixpoint: a box is frozen when every push direction is blocked by
  // a wall, the domain edge, or another frozen box (as target or as
  // the square the player must stand on).
  frozenDead(boxes: number[]): boolean {
    const occ = new Set<number>(boxes);
    const frozen = new Set<number>();
    let changed = true;
    while (changed) {
      changed = false;
      for (const b of boxes) {
        if (frozen.has(b)) continue;
        let allBlocked = true;
        for (let d = 0; d < 4; d++) {
          const t = this.nbr[b][d];
          const s = this.nbr[b][OPP[d]];
          const tBlocked =
            t < 0 || t >= this.n || (occ.has(t) && frozen.has(t));
          const sBlocked =
            s < 0 || (s < this.n && occ.has(s) && frozen.has(s));
          if (!tBlocked && !sBlocked) {
            allBlocked = false;
            break;
          }
        }
        if (allBlocked) {
          frozen.add(b);
          changed = true;
        }
      }
    }
    for (const b of frozen) if (!this.goalSet[b]) return true;
    return false;
  }

  // Flood fill from `from` over room cells + entry, avoiding boxes.
  reachable(from: number, occ: Uint8Array): Uint8Array {
    const seen = new Uint8Array(this.n + 1);
    seen[from] = 1;
    const q: number[] = [from];
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (let d = 0; d < 4; d++) {
        const nb = this.nbr[c][d];
        if (nb < 0 || seen[nb] || occ[nb]) continue;
        seen[nb] = 1;
        q.push(nb);
      }
    }
    return seen;
  }

  isGoal(boxes: number[]): boolean {
    for (const b of boxes) if (!this.goalSet[b]) return false;
    return true;
  }

  // Exit check: from the player's cell, can the player walk to the door?
  canExit(player: number, boxes: number[]): boolean {
    const occ = new Uint8Array(this.n + 1);
    for (const b of boxes) occ[b] = 1;
    const seen = this.reachable(player, occ);
    return seen[this.room.doorIdx] === 1;
  }
}

// Deterministic binary heap ordered by (f, h, insertion sequence).
class Heap {
  a: number[] = [];
  f: number[];
  h: number[];
  seq: number[];
  constructor(f: number[], h: number[], seq: number[]) {
    this.f = f;
    this.h = h;
    this.seq = seq;
  }
  get size(): number {
    return this.a.length;
  }
  less(x: number, y: number): boolean {
    if (this.f[x] !== this.f[y]) return this.f[x] < this.f[y];
    if (this.h[x] !== this.h[y]) return this.h[x] < this.h[y];
    return this.seq[x] < this.seq[y];
  }
  push(id: number): void {
    const a = this.a;
    a.push(id);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(a[i], a[p])) break;
      const t = a[i];
      a[i] = a[p];
      a[p] = t;
      i = p;
    }
  }
  pop(): number {
    const a = this.a;
    const top = a[0];
    const last = a.pop() as number;
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && this.less(a[l], a[m])) m = l;
        if (r < a.length && this.less(a[r], a[m])) m = r;
        if (m === i) break;
        const t = a[i];
        a[i] = a[m];
        a[m] = t;
        i = m;
      }
    }
    return top;
  }
}

function packBoxes(boxes: number[]): number {
  let p = 63;
  for (let i = boxes.length - 1; i >= 0; i--) p = (p << 6) | boxes[i];
  return p;
}

interface SearchResult {
  status: 'solved' | 'budget' | 'exhausted';
  pushes: { from: number; dir: number }[];
  endPlayer: number;
  endBoxes: number[];
}

// A* (greedy=false, f = g + h) or greedy best-first (greedy=true,
// f = h) over pushes. `requireExit` adds "player can reach the door"
// to the goal test. Returns the push list on success.
function searchRoom(
  rs: RoomSolver,
  budget: number,
  greedy: boolean,
  requireExit: boolean,
): SearchResult {
  const n = rs.n;
  const startBoxes = rs.room.boxes.slice();
  const startPlayer = rs.room.entryIdx;
  const h0 = rs.heuristic(startBoxes);
  const keys: number[] = [];
  const gScore: number[] = [];
  const hVal: number[] = [];
  const fVal: number[] = [];
  const seqVal: number[] = [];
  const parent: number[] = [];
  const pushFrom: number[] = [];
  const pushDir: number[] = [];
  const boxesOf: number[][] = [];
  const playerOf: number[] = [];
  const closed = new Map<number, number>();
  let seq = 0;
  const addNode = (
    boxes: number[],
    player: number,
    g: number,
    h: number,
    par: number,
    from: number,
    dir: number,
  ): number => {
    const id = keys.length;
    keys.push(packBoxes(boxes) | (player << 24));
    gScore.push(g);
    hVal.push(h);
    fVal.push(greedy ? h : g + h);
    seqVal.push(seq++);
    parent.push(par);
    pushFrom.push(from);
    pushDir.push(dir);
    boxesOf.push(boxes);
    playerOf.push(player);
    return id;
  };
  if (h0 === Infinity || rs.frozenDead(startBoxes)) {
    return { status: 'exhausted', pushes: [], endPlayer: startPlayer, endBoxes: startBoxes };
  }
  // Canonicalise the player position: any two player cells connected by
  // free floor (with these boxes) are equivalent, so the state keeps
  // the smallest cell of the player's reachable region. This merges
  // states that differ only by free walking and shrinks the search.
  const canonicalPlayer = (player: number, boxes: number[]): number => {
    const occ = new Uint8Array(n + 1);
    for (const b of boxes) occ[b] = 1;
    const region = rs.reachable(player, occ);
    for (let i = 0; i <= n; i++) if (region[i]) return i;
    return player;
  };
  const startCanon = canonicalPlayer(startPlayer, startBoxes);
  const startId = addNode(startBoxes, startCanon, 0, h0, -1, -1, -1);
  closed.set(keys[startId], 0);
  const heap = new Heap(fVal, hVal, seqVal);
  heap.push(startId);
  let expansions = 0;
  while (heap.size > 0) {
    const id = heap.pop();
    if (gScore[id] > (closed.get(keys[id]) as number)) continue;
    const boxes = boxesOf[id];
    const player = playerOf[id];
    if (rs.isGoal(boxes) && (!requireExit || rs.canExit(player, boxes))) {
      const pushes: { from: number; dir: number }[] = [];
      let cur = id;
      while (parent[cur] >= 0) {
        pushes.push({ from: pushFrom[cur], dir: pushDir[cur] });
        cur = parent[cur];
      }
      pushes.reverse();
      // The stored player is the canonical region representative; the
      // true final position is where the last push left the player
      // (the pushed box's old cell), or the entry when no push happened.
      const endPlayer =
        pushes.length > 0 ? pushes[pushes.length - 1].from : startPlayer;
      return { status: 'solved', pushes, endPlayer, endBoxes: boxes };
    }
    if (expansions >= budget || globalExpansions >= GLOBAL_EXPANSION_CAP) {
      return { status: 'budget', pushes: [], endPlayer: player, endBoxes: boxes };
    }
    expansions++;
    globalExpansions++;
    const occ = new Uint8Array(n + 1);
    for (const b of boxes) occ[b] = 1;
    const reach = rs.reachable(player, occ);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (let d = 0; d < 4; d++) {
        const pusher = rs.nbr[b][OPP[d]];
        const target = rs.nbr[b][d];
        if (pusher < 0 || !reach[pusher]) continue;
        if (target < 0 || target >= n || occ[target]) continue;
        if (rs.dead[target]) continue;
        const nb: number[] = [];
        for (let k = 0; k < boxes.length; k++) nb.push(k === bi ? target : boxes[k]);
        nb.sort((a, c) => a - c);
        const h = rs.heuristic(nb);
        if (h === Infinity) continue;
        if (rs.frozenDead(nb)) continue;
        const ng = gScore[id] + 1;
        const canon = canonicalPlayer(b, nb);
        const key = packBoxes(nb) | (canon << 24);
        const prev = closed.get(key);
        if (prev !== undefined && prev <= ng) continue;
        closed.set(key, ng);
        heap.push(addNode(nb, canon, ng, h, id, b, d));
      }
    }
  }
  return { status: 'exhausted', pushes: [], endPlayer: startPlayer, endBoxes: startBoxes };
}

// ------------------------------------------------- solving one room, moves ---

interface RoomSolution {
  moves: string;
  endPlayerGrid: number;
  endBoxesGrid: number[];
}

// Turn a push list into player moves, walking between pushes with a
// shortest path (fixed neighbour order, so deterministic).
function pushesToMoves(rs: RoomSolver, pushes: { from: number; dir: number }[]): string {
  const n = rs.n;
  let player = rs.room.entryIdx;
  let boxes = rs.room.boxes.slice();
  let moves = '';
  const bfsPath = (from: number, to: number, occ: Uint8Array): number[] | null => {
    if (from === to) return [];
    const prev = new Int32Array(n + 1).fill(-1);
    const prevDir = new Int8Array(n + 1).fill(-1);
    prev[from] = from;
    const q: number[] = [from];
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (let d = 0; d < 4; d++) {
        const nb = rs.nbr[c][d];
        if (nb < 0 || prev[nb] >= 0 || occ[nb]) continue;
        prev[nb] = c;
        prevDir[nb] = d;
        if (nb === to) {
          const path: number[] = [];
          let cur = to;
          while (cur !== from) {
            path.push(prevDir[cur]);
            cur = prev[cur];
          }
          path.reverse();
          return path;
        }
        q.push(nb);
      }
    }
    return null;
  };
  for (const p of pushes) {
    const occ = new Uint8Array(n + 1);
    for (const b of boxes) occ[b] = 1;
    const pusher = rs.nbr[p.from][OPP[p.dir]];
    const path = bfsPath(player, pusher, occ);
    if (path === null) return '';
    for (const d of path) moves += STEP[d];
    moves += STEP[p.dir];
    const target = rs.nbr[p.from][p.dir];
    boxes = boxes.map((b) => (b === p.from ? target : b)).sort((a, b) => a - b);
    player = p.from;
  }
  return moves;
}

function solveRoom(rs: RoomSolver, level: Level): RoomSolution | null {
  const attempts: { greedy: boolean; exit: boolean }[] = [
    { greedy: false, exit: false },
    { greedy: false, exit: true },
    { greedy: true, exit: true },
    { greedy: true, exit: false },
  ];
  for (const a of attempts) {
    const budget = a.greedy ? GREEDY_BUDGET_PER_ROOM : ASTAR_BUDGET_PER_ROOM;
    const r = searchRoom(rs, budget, a.greedy, a.exit);
    if (r.status !== 'solved') continue;
    if (!a.exit && !rs.canExit(r.endPlayer, r.endBoxes)) continue;
    const moves = pushesToMoves(rs, r.pushes);
    if (moves === '' && r.pushes.length > 0) continue;
    return {
      moves,
      endPlayerGrid: rs.room.cells[r.endPlayer] ?? rs.room.entry,
      endBoxesGrid: r.endBoxes.map((b) => rs.room.cells[b]),
    };
  }
  return null;
}

// ------------------------------------------- walking on the whole level ---

// Shortest player walk on the full level avoiding the given boxes
// (grid indices). Fixed neighbour order: deterministic.
function walkFull(level: Level, from: number, to: number, boxAt: Set<number>): string | null {
  if (from === to) return '';
  const { w, h, wall } = level;
  const prev = new Int32Array(w * h).fill(-1);
  const prevDir = new Int8Array(w * h).fill(-1);
  prev[from] = from;
  const q: number[] = [from];
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const y = Math.floor(c / w);
    const x = c % w;
    for (let d = 0; d < 4; d++) {
      const ny = y + DY[d];
      const nx = x + DX[d];
      if (ny < 0 || ny >= h || nx < 0 || nx >= w) continue;
      const nc = ny * w + nx;
      if (wall[nc] || prev[nc] >= 0 || boxAt.has(nc)) continue;
      prev[nc] = c;
      prevDir[nc] = d;
      if (nc === to) {
        let moves = '';
        let cur = to;
        const rev: number[] = [];
        while (cur !== from) {
          rev.push(prevDir[cur]);
          cur = prev[cur];
        }
        rev.reverse();
        for (const dd of rev) moves += STEP[dd];
        return moves;
      }
      q.push(nc);
    }
  }
  return null;
}

// Replay a move string on the level: every step must be legal and the
// final position must have every box on a goal. Returns push count,
// or -1 when illegal / unsolved.
function replay(level: Level, moves: string): number {
  const { w, wall, goals } = level;
  const boxAt = new Set<number>(level.boxes);
  let player = level.player;
  let pushes = 0;
  for (const ch of moves) {
    const d = STEP.indexOf(ch.toLowerCase());
    if (d < 0) {
      if (ch === '\n' || ch === ' ' || ch === '\r' || ch === '\t') continue;
      return -1;
    }
    const y = Math.floor(player / w);
    const x = player % w;
    const ny = y + DY[d];
    const nx = x + DX[d];
    if (ny < 0 || ny >= level.h || nx < 0 || nx >= w) return -1;
    const nc = ny * w + nx;
    if (wall[nc]) return -1;
    if (boxAt.has(nc)) {
      const by = ny + DY[d];
      const bx = nx + DX[d];
      if (by < 0 || by >= level.h || bx < 0 || bx >= w) return -1;
      const bc = by * w + bx;
      if (wall[bc] || boxAt.has(bc)) return -1;
      boxAt.delete(nc);
      boxAt.add(bc);
      pushes++;
    }
    player = nc;
  }
  for (const b of boxAt) if (!goals[b]) return -1;
  return pushes;
}

function solveLevel(level: Level): string {
  const rooms = findRooms(level);
  if (rooms.length === 0) return '';
  const boxAt = new Set<number>(level.boxes);
  let player = level.player;
  let moves = '';
  for (const room of rooms) {
    const walk = walkFull(level, player, room.entry, boxAt);
    if (walk === null) return '';
    moves += walk;
    player = room.entry;
    const rs = new RoomSolver(room, level);
    const sol = solveRoom(rs, level);
    if (sol === null) return '';
    moves += sol.moves;
    for (const b of room.boxes.map((bi) => room.cells[bi])) boxAt.delete(b);
    for (const b of sol.endBoxesGrid) boxAt.add(b);
    player = sol.endPlayerGrid;
  }
  if (replay(level, moves) < 0) return '';
  return moves;
}

for (const id of readdirSync(INPUT_DIR).sort()) {
  let moves = '';
  try {
    const level = parseLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'));
    moves = solveLevel(level);
  } catch {
    moves = '';
  }
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), moves + '\n');
}
