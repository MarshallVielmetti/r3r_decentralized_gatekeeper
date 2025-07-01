#include "st_rrt_impl.hpp"

#include <array>
#include <iostream>
#include <vector>

auto run() -> int {
  // Initialize the planner parameters
  initialize_planner(100, 1.0, 1.0, 0.5);

  std::cout << "Testing C++ plan_strrt function...\n";

  // Define start and goal states
  std::array<double, 3> start_state = {0.0, 0.0, 28.0}; // x, y, t
  std::array<double, 2> goal_state = {90.0, 90.0};      // x, y

  auto is_valid_func = [](double time, double pos_x, double pos_y) -> bool {
    // Simple validity function: avoid a small obstacle region
    return (pos_x < 80.0 || pos_x > 85.0 || pos_y < 80.0 || pos_y > 85.0) &&
           time >= 0;
  };

  // Call the C++ planning function
  auto result = plan_strrt(start_state, goal_state, is_valid_func);

  if (!result.has_value()) {
    std::cout << "Planning FAILED - no solution found\n";
    return 1;
  }

  std::cout << "Result had a value!\n";

  // Get the waypoints vector
  const auto &waypoints = result.value();

  // Print the waypoints
  for (size_t i = 0; i < waypoints.size(); i++) {
    const auto &waypoint = waypoints[i];
    std::cout << "Waypoint [" << i << "] - x: " << waypoint[0]
              << ", y: " << waypoint[1] << ", t: " << waypoint[2] << '\n';
  }

  return 0;
}

auto main() -> int {
  for (size_t i = 0; i < 1000; i++) {
    run();
  }
}