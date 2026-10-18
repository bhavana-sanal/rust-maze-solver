# rust-maze-solver

[![CI](https://github.com/bhavana-sanal/rust-maze-solver/actions/workflows/ci.yml/badge.svg)](https://github.com/bhavana-sanal/rust-maze-solver/actions) ![License](https://img.shields.io/badge/license-MIT-blue)

Generates random mazes and solves them in the terminal, comparing **BFS** with **A\***.

- Generation: recursive backtracker (depth-first), producing a perfect maze
- Solvers: breadth-first search and A* with a Manhattan heuristic
- Prints the solved maze and how many cells each algorithm expanded

```bash
cargo run --release -- 25 10 42   # width, height (in cells), seed
cargo test
```

## Tests
Verifies that generated mazes are perfect (spanning trees) and that BFS and A* always return the same shortest path length.
