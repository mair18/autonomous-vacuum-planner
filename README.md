# Autonomous Vacuum Planner — Hybrid CSP & Graph Search Navigation

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-orange.svg)](https://jupyter.org/)
[![AI Paradigm](https://img.shields.io/badge/AI-CSP%20%2B%20A*%20Search-brightgreen.svg)](#system-architecture)

An autonomous indoor vacuum robot planning system that integrates **Constraint Satisfaction Problem (CSP)** temporal scheduling with **cost-aware graph search pathfinding**. Given a 2D grid environment with obstacles, dirty target cells, and temporal ordering/deadline rules, the planner computes a globally optimal, collision-free cleaning tour starting and ending at the charging station.

---

## System Architecture

The planner operates as a two-stage hybrid optimization pipeline:

1. **Temporal Task Scheduling (CSP Layer):**
   - **Variables & Domains:** Maps dirty locations ($D_1 \dots D_k$) to discrete time slots ($T_1 \dots T_N$).
   - **Constraints:** Enforces slot exclusivity (*AllDiff*), binary precedence rules ($D_i \prec D_j$), and unary latest-slot deadlines ($D_i \le T_k$).
   - **Heuristics & Inference:** Combines **Minimum Remaining Values (MRV)**, **Degree Heuristic**, **Least Constraining Value (LCV)**, and **Forward Checking** to prune invalid branches early and achieve zero-backtrack solving.

2. **Cost-Aware Tour & Path Optimization (Search Layer):**
   - **All-Pairs Shortest Path Caching:** Computes exact grid distances between the charging station and all dirty cells using admissible Manhattan-heuristic $A^*$.
   - **Global Tour Ranking:** Evaluates feasible CSP schedules against physical travel distances to select the minimum-step closed-loop tour rather than stopping at an arbitrary valid schedule.
   - **Multi-Algorithm Pathfinding:** Implements and benchmarks **BFS**, **DFS**, **Greedy Best-First Search (GBFS)**, **$A^*$ (Manhattan)**, **$A^*$ (Euclidean)**, and **$A^*$ (Zero / Uniform-Cost)**.

---

## Key Empirical Results

### 1. CSP Solver Ablation ($12 \times 12$ Grid, 7 Slots, 6 Tasks)
| Solver Configuration | Recursive Calls | Constraint Checks | Backtracks | Pruned Values |
| :--- | :---: | :---: | :---: | :---: |
| Vanilla Backtracking (No Heuristics) | 14 | 58 | 7 | 0 |
| **MRV + Degree + LCV + Forward Checking** | **7** | **6** | **0** | **15** |

### 2. Standard CSP vs. Cost-Aware Hybrid Optimization
Out of **300** valid CSP schedules, selecting a schedule without spatial distance awareness incurs unnecessary backtracking across the map:
| Selection Strategy | Execution Sequence | Total Tour Cost | Nodes Expanded |
| :--- | :--- | :---: | :---: |
| Standard CSP (First Valid Hit) | `D1(T1) → D4(T2) → D2(T3) → D3(T4) → D5(T5) → D6(T6)` | 80 steps | 246 |
| **Cost-Aware Hybrid Optimal** | **`D1(T1) → D4(T2) → D6(T3) → D2(T4) → D3(T6) → D5(T7)`** | **64 steps** | **126** |

### 3. Graph Search Benchmark on Optimal 7-Leg Tour
| Algorithm | Tour Cost (Steps) | Cumulative Nodes Expanded | Optimality |
| :--- | :---: | :---: | :--- |
| **A\* (Manhattan)** | **64** | **126** | **Optimal (Admissible $L_1$)** |
| A\* (Euclidean) | 64 | 134 | Optimal (Admissible $L_2$) |
| A\* (Zero / UCS) | 64 | 343 | Optimal (Uninformed) |
| BFS | 64 | 346 | Optimal (Unit Step Cost) |
| GBFS | 72 | 91 | Suboptimal (+8 steps) |
| DFS | 338 | 457 | Suboptimal (+274 steps) |

---

## Repository Structure

```text
autonomous-vacuum-planner/
├── autonomous_vacuum_planner.ipynb       # Complete implementation, benchmarks & visualizations
├── Autonomous_Vacuum_Planner_Report.pdf  # Full technical report & complexity analysis
├── Environment.txt                       # Primary 12x12 test environment (25 obstacles, 6 targets)
├── Schedule.txt                          # Primary temporal schedule (7 slots, 3 precedence, 2 deadlines)
├── Environment_Sample.txt                # Benchmark 8x8 environment (6 obstacles, 4 targets)
├── Schedule_Sample.txt                   # Benchmark 4-slot schedule constraints
├── Environment_Error.txt                 # Failure test case: spatially sealed target D2 at (9,8)
└── Schedule_Error.txt                    # Failure test case: cyclic precedence (D1 -> D3 -> D2 -> D1)
```

---

## Getting Started

### Prerequisites
Install the required scientific Python libraries:
```bash
pip install numpy pandas matplotlib jupyter
```

### Running the Planner
1. Clone the repository:
   ```bash
   git clone [https://github.com/mair18/autonomous-vacuum-planner.git](https://github.com/mair18/autonomous-vacuum-planner.git)
   cd autonomous-vacuum-planner
   ```
2. Launch Jupyter Notebook:
   ```bash
   jupyter notebook autonomous_vacuum_planner.ipynb
   ```
3. Run all cells (`Kernel -> Restart & Run All`) to reproduce:
   - Environment & schedule parsing diagnostics
   - CSP heuristic ablation & global schedule ranking
   - Multi-algorithm trajectory plots, U-trap exploration heatmaps, and performance charts
   - $8 \times 8$ sample benchmark verification
   - Graceful failure detection for unreachable cells and circular dependencies

---

## Input File Formats

### `Environment.txt`
```text
12,12                          # Grid dimensions (rows, cols)
25                             # Number of obstacle cells
(2,2), (2,3), ...              # Obstacle coordinates
6                              # Number of dirty cells (D1..D6)
(3,3), (9,8), ...              # Dirty cell coordinates
(0,0)                          # Charging station coordinate
```

### `Schedule.txt`
```text
7                              # Total available time slots (T1..T7)
3                              # Number of binary precedence constraints
(D1,D3)                        # D1 must be cleaned strictly before D3
(D4,D2)
(D2,D5)
2                              # Number of unary deadline constraints
(D1,T3)                        # D1 must be cleaned no later than slot T3
(D4,T5)
```

---

## Group Members

| Roll Number | Name |
| :--- | :--- |
| **CS-24023** | Syed Ramish Akhter |
| **CS-24025** | Syed Arshman Ali Shah |
| **CS-24026** | Mair Nasir |
| **CS-24034** | Siraj-ul-Haq |
