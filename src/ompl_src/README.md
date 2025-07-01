# OMPL Space-Time Motion Planning Library

A C++ motion planning library using OMPL (Open Motion Planning Library) with space-time planning capabilities, featuring ST-RRT* planner implementation and Julia interface support.

## Features

- **Space-Time Planning**: 3D planning in (x, y, time) coordinates
- **Dynamic Obstacle Avoidance**: Support for moving obstacles via validity checking functions
- **ST-RRT* Planner**: Asymptotically optimal motion planning
- **C/C++ API**: Native C++ interface with C bindings
- **Julia Interface**: Full Julia compatibility using `ccall`
- **Memory Safety**: AddressSanitizer support for development
- **Cross-Platform**: macOS, Linux support

## Dependencies

### macOS (Homebrew)
```bash
brew install ompl boost eigen cmake
```

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install libompl-dev libboost-all-dev libeigen3-dev cmake
```

## Building

### Quick Start (Default with AddressSanitizer)
```bash
mkdir build && cd build
cmake ..
make
```

### Build Options

#### Build WITH AddressSanitizer (Default)
Recommended for development and debugging:
```bash
cd build
cmake ..
make
```

#### Build WITHOUT AddressSanitizer
Required for Julia interface and production use:
```bash
cd build
cmake -DENABLE_ASAN=OFF ..
make
```

#### Release Build
```bash
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_ASAN=OFF ..
make
```

#### Disable Tests
```bash
cd build
cmake -DBUILD_TESTS=OFF ..
make
```

## Testing

### C++ Tests
```bash
cd build
./test_cpp_planning_fixed
```

### Julia Interface
**Important**: Julia interface requires ASAN disabled:
```bash
cd build
cmake -DENABLE_ASAN=OFF ..
make
cd ../test
julia test_julia_interface.jl
```

## Usage

### C++ API
```cpp
#include "st_rrt_impl.hpp"

// Initialize planner
initialize_planner(100, 1.0f, 1.0f, 0.5f);

// Define start and goal
std::array<double, 3> start = {10.0, 10.0, 0.0};  // x, y, t
std::array<double, 2> goal = {80.0, 80.0};         // x, y

// Define validity checker
auto is_valid = [](double time, double x, double y) -> bool {
    // Add obstacle checking logic here
    return true;  // No obstacles
};

// Plan path
auto result = plan_strrt(start, goal, is_valid);
if (result.has_value()) {
    for (const auto& waypoint : result.value()) {
        std::cout << "x=" << waypoint[0] << ", y=" << waypoint[1] 
                  << ", t=" << waypoint[2] << std::endl;
    }
}
```

### Julia API
```julia
# Load library
const LIB_PATH = "../build/libompl_src.dylib"

# Initialize planner
ccall((:init, LIB_PATH), Cvoid, (Cint, Cfloat, Cfloat, Cfloat),
      100, 1.0f0, 1.0f0, 0.5f0)

# Define validity checker
function isValid(time::Float64, x::Float64, y::Float64)::Cint
    # Add obstacle checking logic
    return Cint(1)  # Valid
end

# Create C function pointer
c_is_valid = @cfunction(isValid, Cint, (Cdouble, Cdouble, Cdouble))

# Plan path
current_state = [10.0, 10.0, 0.0]
goal_state = [80.0, 80.0]
path_out = zeros(Float64, 300)  # 100 waypoints * 3
actual_length = Ref{Cint}(0)

success = ccall((:plan_strrt_julia, LIB_PATH), Cint,
                (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cvoid}, Ptr{Cdouble}, Cint, Ptr{Cint}),
                current_state, goal_state, c_is_valid, path_out, 100, actual_length)
```

## Configuration

### Planner Parameters
- `space_dim`: Spatial dimensions (e.g., 100 for 100x100 grid)
- `delta`: Collision avoidance distance
- `v_max`: Maximum velocity
- `t_weight`: Time weight in cost function

### Build Configuration
- `ENABLE_ASAN`: Enable/disable AddressSanitizer (default: ON)
- `BUILD_TESTS`: Enable/disable test builds (default: ON)
- `CMAKE_BUILD_TYPE`: Debug or Release (default: Debug)

## Memory Safety

### AddressSanitizer
When `ENABLE_ASAN=ON` (default):
- Detects buffer overflows, use-after-free, memory leaks
- Enabled only in Debug builds
- **Must be disabled for Julia interface**

### Usage Guidelines
- **Development**: Use ASAN enabled for C++ development
- **Julia Integration**: Always disable ASAN (`-DENABLE_ASAN=OFF`)
- **Production**: Use Release builds without ASAN

## File Structure
```
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── include/
│   └── st_rrt_impl.hpp     # Header declarations
├── src/
│   ├── st_rrt_impl.cpp     # Core planning implementation
│   └── ompl_bindings.cpp   # C interface wrapper
├── test/
│   ├── test_cpp_planning_fixed.cpp    # C++ test
│   └── test_julia_interface.jl        # Julia test
└── build/                  # Build output
    └── libompl_src.dylib   # Shared library
```

## Troubleshooting

### AddressSanitizer Issues
- **Julia crashes**: Disable ASAN with `-DENABLE_ASAN=OFF`
- **Interceptors not working**: This is expected with dynamic loading

### Build Issues
- **OMPL not found**: Check Homebrew installation paths in CMakeLists.txt
- **C++17 errors**: Ensure modern compiler (GCC 7+, Clang 5+)

### Planning Issues
- **No solution found**: Increase time limit or adjust planner range
- **Invalid states**: Check validity checker implementation

## License

[Add your license here]

## Contributing

[Add contribution guidelines here]