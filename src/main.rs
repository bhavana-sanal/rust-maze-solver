use maze_solver::{Maze, Rng};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num = |i: usize, d: u64| args.get(i).and_then(|s| s.parse().ok()).unwrap_or(d);
    let (w, h, seed) = (num(1, 20) as usize, num(2, 8) as usize, num(3, 42));
    let maze = Maze::generate(w, h, &mut Rng::new(seed));
    let (path, expanded_astar) = maze.solve_astar().expect("maze is always solvable");
    let (_, expanded_bfs) = maze.solve_bfs().unwrap();
    print!("{}", maze.render(&path));
    println!("path length {} | cells expanded: A* {} vs BFS {}", path.len(), expanded_astar, expanded_bfs);
}
