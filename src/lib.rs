//! Perfect-maze generation and shortest-path solving.
use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self { Rng(seed.max(1)) }
    pub fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 % n as u64) as usize
    }
}

/// Grid of cells; `true` = wall. Dimensions are always odd so walls sit between cells.
pub struct Maze {
    pub w: usize,
    pub h: usize,
    pub walls: Vec<bool>,
}

type P = (usize, usize);

impl Maze {
    /// Recursive-backtracker (iterative DFS) generation: a perfect maze with exactly one path between any two cells.
    pub fn generate(cells_w: usize, cells_h: usize, rng: &mut Rng) -> Maze {
        let (w, h) = (cells_w * 2 + 1, cells_h * 2 + 1);
        let mut m = Maze { w, h, walls: vec![true; w * h] };
        let mut stack = vec![(1usize, 1usize)];
        m.walls[w + 1] = false;
        while let Some(&(x, y)) = stack.last() {
            let mut next = Vec::new();
            for (dx, dy) in [(2i32, 0i32), (-2, 0), (0, 2), (0, -2)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx > 0 && ny > 0 && (nx as usize) < w && (ny as usize) < h && m.walls[ny as usize * w + nx as usize] {
                    next.push((nx as usize, ny as usize));
                }
            }
            if next.is_empty() {
                stack.pop();
            } else {
                let (nx, ny) = next[rng.below(next.len())];
                m.walls[((y + ny) / 2) * w + (x + nx) / 2] = false; // knock down the wall between
                m.walls[ny * w + nx] = false;
                stack.push((nx, ny));
            }
        }
        m
    }

    pub fn start(&self) -> P { (1, 1) }
    pub fn goal(&self) -> P { (self.w - 2, self.h - 2) }

    fn neighbours(&self, (x, y): P) -> impl Iterator<Item = P> + '_ {
        [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)].into_iter().filter_map(move |(dx, dy)| {
            let (nx, ny) = (x as i32 + dx, y as i32 + dy);
            let ok = nx >= 0 && ny >= 0 && (nx as usize) < self.w && (ny as usize) < self.h && !self.walls[ny as usize * self.w + nx as usize];
            ok.then_some((nx as usize, ny as usize))
        })
    }

    fn trace(&self, prev: &[Option<P>], mut cur: P) -> Vec<P> {
        let mut path = vec![cur];
        while let Some(p) = prev[cur.1 * self.w + cur.0] {
            path.push(p);
            cur = p;
        }
        path.reverse();
        path
    }

    /// Breadth-first search. Returns (path, cells expanded).
    pub fn solve_bfs(&self) -> Option<(Vec<P>, usize)> {
        let mut prev = vec![None; self.w * self.h];
        let mut seen = vec![false; self.w * self.h];
        let mut q = VecDeque::from([self.start()]);
        seen[self.start().1 * self.w + self.start().0] = true;
        let mut expanded = 0;
        while let Some(c) = q.pop_front() {
            expanded += 1;
            if c == self.goal() { return Some((self.trace(&prev, c), expanded)); }
            for n in self.neighbours(c) {
                if !seen[n.1 * self.w + n.0] {
                    seen[n.1 * self.w + n.0] = true;
                    prev[n.1 * self.w + n.0] = Some(c);
                    q.push_back(n);
                }
            }
        }
        None
    }

    /// A* with the Manhattan-distance heuristic. Returns (path, cells expanded).
    pub fn solve_astar(&self) -> Option<(Vec<P>, usize)> {
        let g = self.goal();
        let h = |p: P| p.0.abs_diff(g.0) + p.1.abs_diff(g.1);
        let mut best = vec![usize::MAX; self.w * self.h];
        let mut prev = vec![None; self.w * self.h];
        let mut heap = BinaryHeap::new();
        let s = self.start();
        best[s.1 * self.w + s.0] = 0;
        heap.push(Reverse((h(s), 0usize, s)));
        let mut expanded = 0;
        while let Some(Reverse((_, cost, c))) = heap.pop() {
            if cost > best[c.1 * self.w + c.0] { continue; }
            expanded += 1;
            if c == g { return Some((self.trace(&prev, c), expanded)); }
            for n in self.neighbours(c) {
                if cost + 1 < best[n.1 * self.w + n.0] {
                    best[n.1 * self.w + n.0] = cost + 1;
                    prev[n.1 * self.w + n.0] = Some(c);
                    heap.push(Reverse((cost + 1 + h(n), cost + 1, n)));
                }
            }
        }
        None
    }

    pub fn render(&self, path: &[P]) -> String {
        let mut out = String::new();
        for y in 0..self.h {
            for x in 0..self.w {
                out.push(if self.walls[y * self.w + x] { '#' } else if path.contains(&(x, y)) { '*' } else { ' ' });
            }
            out.push('\n');
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maze_is_perfect_and_solvable() {
        let m = Maze::generate(15, 10, &mut Rng::new(5));
        // A perfect maze over N cells has N-1 passages, so open squares = cells + passages = 2N - 1.
        let open = m.walls.iter().filter(|w| !**w).count();
        assert_eq!(open, 2 * 15 * 10 - 1);
        assert!(m.solve_bfs().is_some());
    }

    #[test]
    fn bfs_and_astar_agree_on_shortest_length() {
        for seed in 1..30 {
            let m = Maze::generate(12, 12, &mut Rng::new(seed));
            let (b, be) = m.solve_bfs().unwrap();
            let (a, ae) = m.solve_astar().unwrap();
            assert_eq!(b.len(), a.len(), "seed {seed}");
            assert_eq!(a[0], m.start());
            assert_eq!(*a.last().unwrap(), m.goal());
            assert!(ae <= be, "A* expanded more than BFS");
        }
    }
}
