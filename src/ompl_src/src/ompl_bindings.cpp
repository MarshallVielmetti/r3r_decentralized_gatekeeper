
#include "st_rrt_impl.hpp"
#include <functional>
#include <vector>

extern "C" {

void init(int space_dim, float delta, float v_max, float t_weight) {
  initialize_planner(space_dim, delta, v_max, t_weight);
}

auto plan_strrt_julia(const double *current_state, const double *goal_state,
                      IsValidC is_valid_julia, double *path_out,
                      int max_path_length, int *actual_path_length) -> int {

  *actual_path_length = 0; // Initialize output length

  try {
    // Convert C arrays to C++ arrays
    std::array<double, 3> start_state = {current_state[0], current_state[1],
                                         current_state[2]};

    std::array<double, 2> goal_state_arr = {goal_state[0], goal_state[1]};

    // double start_time_offset = start_state[2];

    // TODO -- Need to verify how the start_time_offset works
    std::function<bool(double, double, double)> is_valid_func =
        [is_valid_julia](double time, double agent_x, double agent_y) -> bool {
      return static_cast<bool>(is_valid_julia(time, agent_x, agent_y));
    };

    // Call the C++ planning function
    auto result = plan_strrt(start_state, goal_state_arr, is_valid_func);

    if (!result.has_value()) {
      *actual_path_length = 0;
      return 0; // Planning failed
    }

    const auto &path = result.value();

    // Check if we have enough space in the output array
    if (path.size() >= max_path_length) {
      *actual_path_length = 0;
      return 0; // Not enough space
    }

    *actual_path_length = static_cast<int>(path.size());

    // Copy path data to output array
    // Format: [x1, y1, t1, x2, y2, t2, ...]
    for (int i = 0; i < path.size(); ++i) {
      path_out[(i * 3) + 0] = path[i][0]; // x
      path_out[(i * 3) + 1] = path[i][1]; // y
      path_out[(i * 3) + 2] = path[i][2]; // + start_time_offset; // t
    }

    return 1; // Success

    // const auto &path = result.value();
    // int path_length = static_cast<int>(path.size());

    // // Check if we have enough space in the output array
    // if (path_length > max_path_length) {
    //   *actual_path_length = 0;
    //   return 0; // Not enough space
    // }

    // // Copy path data to output array
    // // Format: [x1, y1, t1, x2, y2, t2, ...]
    // for (int i = 0; i < path_length; ++i) {
    //   path_out[(i * 3) + 0] = path[i][0]; // x
    //   path_out[(i * 3) + 1] = path[i][1]; // y
    //   path_out[(i * 3) + 2] = path[i][2]; // + start_time_offset; // t
    // }

    // *actual_path_length = path_length;
    // return 1; // Success

  } catch (...) {
    *actual_path_length = 0;
    return 0; // Exception occurred
  }
}

auto plan_strrt_simple_julia(const double *current_state,
                             const double *goal_state, double *path_out,
                             int max_path_length, int *actual_path_length)
    -> int {
  try {
    // Convert C arrays to C++ arrays
    std::array<double, 3> start_state = {current_state[0], current_state[1],
                                         current_state[2]};
    std::array<double, 2> goal_state_arr = {goal_state[0], goal_state[1]};

    // No obstacles for simple version
    std::vector<std::function<std::shared_ptr<ompl::base::State>(float)>>
        empty_obstacles;

    std::function<bool(double, double, double)> is_valid_func =
        [](double, double, double) -> bool { return true; };

    // Call the C++ planning function
    auto result = plan_strrt(start_state, goal_state_arr, is_valid_func);

    if (!result.has_value()) {
      *actual_path_length = 0;
      return 0; // Planning failed
    }

    const auto &path = result.value();
    return 0;
    // int path_length = static_cast<int>(path.size());

    // // Check if we have enough space in the output array
    // if (path_length > max_path_length) {
    //   *actual_path_length = 0;
    //   return 0; // Not enough space
    // }

    // // Copy path data to output array
    // // Format: [x1, y1, t1, x2, y2, t2, ...]
    // for (int i = 0; i < path_length; ++i) {
    //   path_out[(i * 3) + 0] = path[i][0]; // x
    //   path_out[(i * 3) + 1] = path[i][1]; // y
    //   path_out[(i * 3) + 2] = path[i][2]; // t
    // }

    // *actual_path_length = path_length;
    // return 1; // Success

  } catch (...) {
    *actual_path_length = 0;
    return 0; // Exception occurred
  }
}
}
