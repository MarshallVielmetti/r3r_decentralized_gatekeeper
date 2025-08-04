# R3R: A Decentralized Multi-Agent Motion Planning Framework

This project is a simulation framework written in Julia for decentralized, multi-agent motion planning. It implements the "Gatekeeper" algorithm, which allows multiple agents to navigate to their goals in a shared 2D environment while avoiding collisions with one another.

The core of the pathfinding logic leverages the Spatiotemporal RRT* (ST-RRT*) algorithm, using a custom C++ implementation that interfaces with the Open Motion Planning Library (OMPL).

## Key Features

*   **Decentralized Coordination:** No central controller is required. Agents coordinate with their local neighbors.
*   **Collision-Free Navigation:** Implements a robust strategy based on committed trajectories and backup sets to ensure safety.
*   **High-Performance Path Planning:** Integrates the ST-RRT* algorithm from OMPL for efficient, dynamically-aware pathfinding.
*   **Extensible Agent Models:** The framework supports different types of agents with varying dynamics, such as:
    *   `PathFollower2D`: A simple agent that follows a geometric path.
    *   `DoubleIntegrator2D`: A more complex agent with velocity and acceleration, controlled by a Model Predictive Controller (MPC).
*   **Simulation & Visualization:** Includes tools to run simulations and generate `.gif` animations of the results.

## Core Concepts

The "Gatekeeper" algorithm is built on a few key ideas:

1.  **Joining the Network:** An agent can only begin moving if it can find a safe initial trajectory that doesn't conflict with agents already in the "network".
2.  **Committed Trajectories:** Each agent has a `committed_trajectory` that it broadcasts to its neighbors. This is the path it guarantees it will follow for a certain time horizon.
3.  **Replanning:** Agents must replan a new `candidate_trajectory` before their current one expires. An agent can only commit to a new trajectory if it is validated to be collision-free with respect to its neighbors' committed trajectories.
4.  **Backup Sets:** Each trajectory includes a `backup_set`, which is a safe, pre-planned stopping point. If an agent fails to find a new valid trajectory, it can execute its backup plan to come to a safe stop, preventing collisions.

## Project Structure

```
/
├── Project.toml        # Julia project dependencies
├── src/
│   ├── r3r.jl          # Main project module
│   ├── gatekeeper.jl   # Core logic for the decentralized Gatekeeper algorithm
│   ├── strrt_star.jl   # Julia interface to the C++ ST-RRT* planner
│   ├── ompl_src/       # C++ source for the OMPL-based planner
│   ├── 2d_path_tracker.jl # Agent logic for simple path-following agents
│   ├── double_integrators_gatekeeper.jl # Agent logic for double-integrator agents using MPC
│   └── plot_utils.jl   # Utilities for animating simulations
├── test/               # Unit and integration tests
├── simple_test.jl      # An example script to run a simulation
└── outputs/            # Default directory for saved animations
```

## Getting Started

### Prerequisites

*   Julia (v1.6 or later recommended)
*   A C++ compiler (e.g., GCC, Clang) and CMake to build the OMPL module.

### 1. Build the C++ Planner

The core path planner is in C++ and must be compiled into a shared library first.

```bash
cd src/ompl_src
mkdir -p build
cd build
cmake ..
make
```

This will create a `libompl_src.dylib` (or `.so` on Linux) file in the `build` directory.

### 2. Install Julia Dependencies

Navigate to the project root directory and start the Julia REPL.

```bash
julia
```

Enter the package manager by pressing `]` and activate the project environment.

```julia
pkg> activate .
pkg> instantiate
```

### 3. Run a Simulation

You can run a simulation by executing the `simple_test.jl` file from the root of the project.

```bash
julia simple_test.jl
```

This will run a simulation with the default parameters and save an animation of the result to `agent_animation.gif`.
