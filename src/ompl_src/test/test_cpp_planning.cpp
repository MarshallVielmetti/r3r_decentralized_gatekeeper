#include "st_rrt_impl.hpp"

#include <array>
#include <iostream>
#include <vector>

auto main() -> int {
  // Initialize the planner parameters
  initialize_planner(100, 1.0, 1.0, 0.5);

  std::cout << "Testing C++ plan_strrt function...\n";

  // Define start and goal states
  std::array<double, 3> start_state = {10.0, 10.0, 0.0}; // x, y, t
  std::array<double, 2> goal_state = {90.0, 90.0};       // x, y

  // Create empty obstacle functions vector for this test
  std::vector<std::function<std::shared_ptr<ob::State>(float)>> obstacles;

  auto is_valid_func = [](float t, double x, double y) -> bool {
    // Simple validity function: always valid in this test
    return x < 80.0 || x > 85.0 || y < 80.0 || y > 85.0;
  };

  // Call the C++ planning function
  auto result = plan_strrt(start_state, goal_state, is_valid_func);

  if (result.has_value()) {
    std::cout << "Planning SUCCESS!\n";

    const auto &path = result.value();
    std::cout << "Path contains " << path.size() << " waypoints:\n";

    // Print first few and last few waypoints
    size_t max_display = std::min(static_cast<size_t>(10), path.size());

    for (size_t i = 0; i < max_display; ++i) {
      const auto &waypoint = path[i];
      std::cout << "  [" << i << "] x=" << waypoint[0] << ", y=" << waypoint[1]
                << ", t=" << waypoint[2] << '\n';
    }

    if (path.size() > max_display) {
      std::cout << "  ... (" << (path.size() - max_display)
                << " more waypoints)" << '\n';
    }

    // Verify start and end points
    const auto &first_point = path.front();
    const auto &last_point = path.back();

    std::cout << "\nVerification:" << '\n';
    std::cout << "Start point: (" << first_point[0] << ", " << first_point[1]
              << ", " << first_point[2] << ")" << '\n';
    std::cout << "End point: (" << last_point[0] << ", " << last_point[1]
              << ", " << last_point[2] << ")" << '\n';

    // Check if we reached the goal spatially
    double goal_distance = std::sqrt(
        ((last_point[0] - goal_state[0]) * (last_point[0] - goal_state[0])) +
        ((last_point[1] - goal_state[1]) * (last_point[1] - goal_state[1])));

    std::cout << "Distance to goal: " << goal_distance << '\n';
    std::cout << "Total time: " << last_point[2] << '\n';

  } else {
    std::cout << "Planning FAILED - no solution found" << '\n';
    return 1;
  }

  return 0;
}
