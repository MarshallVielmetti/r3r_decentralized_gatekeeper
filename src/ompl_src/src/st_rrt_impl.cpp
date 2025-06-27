#include "st_rrt_impl.hpp"

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>

#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// static std::shared_ptr<ompl::geometric::STRRTstar> planner;
// static std::shared_ptr<ompl::base::RealVectorStateSpace> state_space;
// static std::shared_ptr<ompl::geometric::SimpleSetup> simple_setup;
// // static std::shared_ptr<ompl::base::SpaceInformation> space_info;
// static ompl::base::SpaceInformationPtr space_info;

static std::shared_ptr<og::STRRTstar> planner;
static std::shared_ptr<og::SimpleSetup> simple_setup;
static ob::SpaceInformationPtr space_info_ptr;

static float V_MAX = 1.0;
static float T_WEIGHT = 0.5;
static int SPACE_DIM = 100;
static float DELTA = 1.0; // inter-agent collision avoidance distance

class ValidityChecker : public ob::StateValidityChecker {
public:
  using ObstacleFunction = std::function<std::shared_ptr<ob::State>(float)>;
  using JuliaIsValidFunction = std::function<bool(float, double, double)>;

  ValidityChecker(const ob::SpaceInformationPtr &state_information,
                  JuliaIsValidFunction julia_is_valid)
      : ob::StateValidityChecker(state_information),
        is_valid_(std::move(julia_is_valid)) {
    // Cache the spatial state space for efficient access
    const auto *space_time_space =
        si_->getStateSpace()->as<ob::SpaceTimeStateSpace>();
    spatial_state_space_ =
        space_time_space->getSubspace(0)->as<ob::RealVectorStateSpace>();
  }

private:
  JuliaIsValidFunction is_valid_;
  const ob::RealVectorStateSpace *spatial_state_space_;

public:
  auto isValid(const ob::State *state) const -> bool override {
    // Downcast state to a SpaceTimeStateSpace
    const auto *complete_state =
        state->as<ob::SpaceTimeStateSpace::StateType>();

    // Extract the spatial component (R^2 space)
    const auto *spatial_state =
        complete_state->as<ob::RealVectorStateSpace::StateType>(0);

    // Extract the time component
    const auto *time_state =
        complete_state->as<ob::TimeStateSpace::StateType>(1);

    // Get time value
    double time_value = time_state->position;

    return this->is_valid_(time_value, spatial_state->values[0],
                           spatial_state->values[1]);

    // // Check dynamic obstacle collisions
    // for (const auto &obstacle_func : dynamic_obstacles_) {
    //   auto obstacle_state_ptr =
    //   obstacle_func(static_cast<float>(time_value)); if (obstacle_state_ptr
    //   == nullptr) {
    //     continue; // Skip if obstacle state is not available
    //   }

    //   // Cast obstacle state to RealVectorStateSpace
    //   const auto *obstacle_2d_state =
    //       obstacle_state_ptr->as<ob::RealVectorStateSpace::StateType>();

    //   // Use the cached spatial state space's distance function
    //   double distance =
    //       spatial_state_space_->distance(spatial_state, obstacle_2d_state);

    //   if (distance < DELTA) {
    //     return false; // Collision detected
    //   }
    // }

    // return true;
  }
};

void initialize_planner(int space_dim, float delta, float v_max,
                        float t_weight) {
  // Set global parameters
  SPACE_DIM = space_dim;
  DELTA = delta;
  V_MAX = v_max;
  T_WEIGHT = t_weight;
}

void init_rrt_star_planner() {
  // Initialize the 2D space information
  auto state_space = std::make_shared<ob::RealVectorStateSpace>(2);

  // Set Bounds for R^2
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(SPACE_DIM);
  state_space->setBounds(bounds);

  // Create the space-time state space
  auto time_state_space =
      std::make_shared<ob::SpaceTimeStateSpace>(state_space, V_MAX, T_WEIGHT);

  simple_setup = std::make_shared<og::SimpleSetup>(state_space);

  planner = std::make_shared<og::STRRTstar>(space_info_ptr);
}

auto plan_strrt(
    const std::array<double, 3> &current_state,
    const std::array<double, 2> &goal_state,
    const std::function<bool(double, double, double)> &is_valid_julia)
    -> std::optional<std::vector<std::array<double, 3>>> {
  // Initialize the 2D spatial state space
  auto spatial_state_space = std::make_shared<ob::RealVectorStateSpace>(2);

  // Set bounds for R^2
  ob::RealVectorBounds spatial_bounds(2);
  spatial_bounds.setLow(0);
  spatial_bounds.setHigh(SPACE_DIM);
  spatial_state_space->setBounds(spatial_bounds);

  // Create the space-time state space
  auto space_time_state_space = std::make_shared<ob::SpaceTimeStateSpace>(
      spatial_state_space, V_MAX, T_WEIGHT);

  // Create space information
  auto space_info =
      std::make_shared<ob::SpaceInformation>(space_time_state_space);

  // Create and set the validity checker with dynamic obstacles
  auto validity_checker =
      std::make_shared<ValidityChecker>(space_info, is_valid_julia);
  space_info->setStateValidityChecker(validity_checker);
  space_info->setup();

  // Create the problem definition
  auto problem_def = std::make_shared<ob::ProblemDefinition>(space_info);

  // Set start state
  ob::ScopedState<ob::SpaceTimeStateSpace> start_state(space_time_state_space);
  auto *start_spatial = start_state->as<ob::RealVectorStateSpace::StateType>(0);
  auto *start_time = start_state->as<ob::TimeStateSpace::StateType>(1);

  start_spatial->values[0] = current_state[0]; // x
  start_spatial->values[1] = current_state[1]; // y
  start_time->position = current_state[2];     // t

  problem_def->addStartState(start_state);

  // Set goal state (for space-time planning, we want to reach the spatial goal
  // at any future time)
  ob::ScopedState<ob::SpaceTimeStateSpace> goal_state_st(
      space_time_state_space);
  auto *goal_spatial =
      goal_state_st->as<ob::RealVectorStateSpace::StateType>(0);
  auto *goal_time = goal_state_st->as<ob::TimeStateSpace::StateType>(1);

  goal_spatial->values[0] = goal_state[0]; // x
  goal_spatial->values[1] = goal_state[1]; // y

  // set goal_time position to be 1.5 times the distance between start and goal
  goal_time->position =
      current_state[2] + 1.1 * space_time_state_space->distance(
                                   start_state.get(), goal_state_st.get());

  // Set goal state with spatial tolerance
  problem_def->setGoalState(goal_state_st, 0.5); // 0.5 spatial tolerance

  // Create ST-RRT* planner
  auto strrt_planner = std::make_shared<og::STRRTstar>(space_info);
  strrt_planner->setProblemDefinition(problem_def);

  // Extension range should be a function of the distance between start and
  // goal, with a max of 10.0
  // auto optimal_range =
  //     std::min(0.1 * space_time_state_space->distance(start_state.get(),
  //                                                     goal_state_st.get()),
  //              10.0);

  auto optimal_range =
      std::min(10.0, 0.1 * std::sqrt(((goal_state[0] - current_state[0]) *
                                      (goal_state[0] - current_state[0])) +
                                     ((goal_state[1] - current_state[1]) *
                                      (goal_state[1] - current_state[1]))));

  std::cout << "Optimal range: " << optimal_range << '\n';

  strrt_planner->setRange(optimal_range);
  strrt_planner->setup();

  // Solve the planning problem with time limit
  // Combine a time limit and an optimality improvement threshold for early
  // termination
  auto termination_condition = ob::plannerOrTerminationCondition(
      ob::timedPlannerTerminationCondition(5.0), // 5 second timeout
      ob::CostConvergenceTerminationCondition(problem_def));

  ob::PlannerStatus status = strrt_planner->solve(termination_condition);

  // Check if planning was successful
  if (status == ob::PlannerStatus::EXACT_SOLUTION ||
      status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {

    // Extract the solution path
    auto solution_path = problem_def->getSolutionPath();
    auto path_geometric =
        std::dynamic_pointer_cast<og::PathGeometric>(solution_path);

    if (path_geometric) {
      // Convert the path to output format
      std::vector<std::array<double, 3>> waypoints;

      for (size_t i = 0; i < path_geometric->getStateCount(); ++i) {
        const auto *state = path_geometric->getState(i);
        const auto *complete_state =
            state->as<ob::SpaceTimeStateSpace::StateType>();

        // Extract spatial and time components
        const auto *spatial_state =
            complete_state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *time_state =
            complete_state->as<ob::TimeStateSpace::StateType>(1);

        std::array<double, 3> waypoint = {
            spatial_state->values[0], // x
            spatial_state->values[1], // y
            time_state->position      // t
        };

        waypoints.push_back(waypoint);
      }

      return waypoints;
    }
  }

  // Return empty optional if planning failed or no path found
  return std::nullopt;
}
