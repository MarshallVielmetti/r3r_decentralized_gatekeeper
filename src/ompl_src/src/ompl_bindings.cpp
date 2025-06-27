
#include "st_rrt_impl.hpp"
#include <memory>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <vector>

extern "C" {

void init(int space_dim, float delta, float v_max, float t_weight) {
  initialize_planner(space_dim, delta, v_max, t_weight);
}

// Helper class to wrap C function pointer for use with C++ std::function
// This version uses shared_ptr for safe memory management
class CObstacleFunctionWrapper {
private:
  ObstacleFunctionC func_;
  static std::shared_ptr<ompl::base::RealVectorStateSpace> obstacle_space_;
  static bool space_initialized_;

public:
  CObstacleFunctionWrapper(ObstacleFunctionC func) : func_(func) {
    if (!space_initialized_) {
      obstacle_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(2);
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(-1000); // Large bounds for obstacle positions
      bounds.setHigh(1000);
      obstacle_space_->setBounds(bounds);
      space_initialized_ = true;
    }
  }

  auto operator()(float time) -> std::shared_ptr<ompl::base::State> {
    double pos_x;
    double pos_y;
    int is_valid;
    func_(time, &pos_x, &pos_y, &is_valid);

    if (!is_valid) {
      return nullptr;
    }

    // Create a scoped state for safe memory management
    auto scoped_state = std::make_shared<
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>(
        obstacle_space_);

    // Set the position values
    (*scoped_state)[0] = pos_x;
    (*scoped_state)[1] = pos_y;

    // Return as shared_ptr to the underlying state
    // The ScopedState manages the lifetime automatically
    return std::shared_ptr<ompl::base::State>(scoped_state,
                                              scoped_state->get());
  }
};

// Static member definitions
std::shared_ptr<ompl::base::RealVectorStateSpace>
    CObstacleFunctionWrapper::obstacle_space_ = nullptr;
bool CObstacleFunctionWrapper::space_initialized_ = false;

auto plan_strrt_julia(const double *current_state, const double *goal_state,
                      IsValidC is_valid_julia, double *path_out,
                      int max_path_length, int *actual_path_length) -> int {
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

    // std::function<bool(double, double, double)> is_valid_func =
    //     [is_valid_julia, start_time_offset](double time, double agent_x,
    //                                         double agent_y) -> bool {
    //   return static_cast<bool>(
    //       is_valid_julia(time + start_time_offset, agent_x, agent_y));
    // };

    // Call the C++ planning function
    auto result = plan_strrt(start_state, goal_state_arr, is_valid_func);

    if (!result.has_value()) {
      *actual_path_length = 0;
      return 0; // Planning failed
    }

    const auto &path = result.value();
    int path_length = static_cast<int>(path.size());

    // Check if we have enough space in the output array
    if (path_length > max_path_length) {
      *actual_path_length = 0;
      return 0; // Not enough space
    }

    // Copy path data to output array
    // Format: [x1, y1, t1, x2, y2, t2, ...]
    for (int i = 0; i < path_length; ++i) {
      path_out[(i * 3) + 0] = path[i][0]; // x
      path_out[(i * 3) + 1] = path[i][1]; // y
      path_out[(i * 3) + 2] = path[i][2]; // + start_time_offset; // t
    }

    *actual_path_length = path_length;
    return 1; // Success

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
    int path_length = static_cast<int>(path.size());

    // Check if we have enough space in the output array
    if (path_length > max_path_length) {
      *actual_path_length = 0;
      return 0; // Not enough space
    }

    // Copy path data to output array
    // Format: [x1, y1, t1, x2, y2, t2, ...]
    for (int i = 0; i < path_length; ++i) {
      path_out[(i * 3) + 0] = path[i][0]; // x
      path_out[(i * 3) + 1] = path[i][1]; // y
      path_out[(i * 3) + 2] = path[i][2]; // t
    }

    *actual_path_length = path_length;
    return 1; // Success

  } catch (...) {
    *actual_path_length = 0;
    return 0; // Exception occurred
  }
}
}
