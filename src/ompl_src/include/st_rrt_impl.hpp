#pragma once

#include <array>
#include <functional>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/PathGeometric.h>
#include <optional>
#include <vector>

// Namespace abbreviations (matching the implementation file)
namespace ob = ompl::base;
namespace og = ompl::geometric;
// namespace og = ompl::geometric;

// Forward declarations
namespace ompl::base {
class State;
}

class ValidityChecker;

// C++ interface functions
void initialize_planner(int space_dim, float delta, float v_max,
                        float t_weight);

// Main planning function
auto plan_strrt(
    const std::array<double, 3> &current_state,
    const std::array<double, 2> &goal_state,
    const std::function<bool(double, double, double)> &is_valid_julia)
    -> std::optional<std::vector<std::array<double, 3>>>;

void geometric_path_to_serialized(const og::PathGeometric &path,
                                  int max_path_length, double *path_out,
                                  int *actual_path_length);

// // C interface
extern "C" {
// Julia-compatible C interface
// using ObstacleFunctionC = void (*)(float, double *, double *, int *);
using IsValidC = int (*)(double, double, double);

// typedef void (*ObstacleFunctionC)(float time, double *x_out, double *y_out,
//                                   int *valid_out);

// Planning function for Julia ccall
// Returns: 1 for success, 0 for failure
// path_out: pre-allocated array to store the path (x1,y1,t1,x2,y2,t2,...)
// max_path_length: maximum number of waypoints that can be stored
// actual_path_length: output parameter for actual number of waypoints found
auto plan_strrt_julia(
    const double *current_state, // [x, y, t] array of size 3
    const double *goal_state,    // [x, y] array of size 2
    IsValidC is_valid_julia,     // function pointer (not pointer to pointer)
    double *path_out,            // output array for path data
    int max_path_length,         // maximum waypoints that can be stored
    int *actual_path_length      // actual number of waypoints found
    ) -> int;

// Simplified version without obstacles for testing
auto plan_strrt_simple_julia(
    const double *current_state, // [x, y, t] array of size 3
    const double *goal_state,    // [x, y] array of size 2
    double *path_out,            // output array for path data
    int max_path_length,         // maximum waypoints that can be stored
    int *actual_path_length      // actual number of waypoints found
    ) -> int;
}
// extern "C" {
// auto ompl_bindings_init() -> int;
// auto ompl_bindings_get_value() -> int;
// void init(int space_dim, float delta, float v_max, float t_weight);
// auto plan_strrt_c(double start_x, double start_y, double start_t, double
// goal_x,
//                   double goal_y) -> int;
// auto get_path_length(double start_x, double start_y, double start_t,
//                      double goal_x, double goal_y) -> int;
// auto get_path_point(double start_x, double start_y, double start_t,
//                     double goal_x, double goal_y, int index, double *x_out,
//                     double *y_out, double *t_out) -> int;
// }