#include "st_rrt_impl.hpp"

#include <algorithm>
#include <complex>
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
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

static float V_MAX = 1.0;
static float T_WEIGHT = 0.5;
static int SPACE_DIM = 100;
static float SAMPLING_RESOLUTION = 0.1;
static float DELTA = 1.0; // inter-agent collision avoidance distance

// TODO TEMP REMOVE
// auto isStateValid(const ob::State *state) -> bool {
//   // extract the space component of the state and cast it to what we expect
//   const auto pos_x = state->as<ob::CompoundState>()
//                          ->as<ob::RealVectorStateSpace::StateType>(0)
//                          ->values[0];
//   const auto pos_y = state->as<ob::CompoundState>()
//                          ->as<ob::RealVectorStateSpace::StateType>(0)
//                          ->values[1];

//   // extract the time component of the state and cast it to what we expect
//   const auto t = state->as<ob::CompoundState>()
//                      ->as<ob::TimeStateSpace::StateType>(1)
//                      ->position;

//   // check validity of state defined by pos & t (e.g. check if constraints
//   are
//   // satisfied)...

//   bool in_exclusion_zone =
//       (pos_x > 20 && pos_x < 30 && pos_y > 20 && pos_y < 30);

//   if (in_exclusion_zone) {
//     return false;
//   }

//   // return a value that is always true
//   return t >= 0 && pos_x < std::numeric_limits<double>::infinity() &&
//          pos_y < std::numeric_limits<double>::infinity();
// }

class ValidityChecker : public ob::StateValidityChecker {
public:
  using JuliaIsValidFunction = std::function<bool(double, double, double)>;

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

    const auto *spatial_state =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(
            0);

    const auto *time_state =
        state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);

    return this->is_valid_(time_state->position, spatial_state->values[0],
                           spatial_state->values[1]);
  }
};

class SpaceTimeMotionValidator : public ob::MotionValidator {
public:
  explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si)
      : MotionValidator(si), vMax_(si_->getStateSpace()
                                       .get()
                                       ->as<ob::SpaceTimeStateSpace>()
                                       ->getVMax()),
        stateSpace_(si_->getStateSpace().get()) {};

  auto checkMotion(const ob::State *s1, const ob::State *s2) const
      -> bool override {
    if (!si_->isValid(s2)) {
      invalid_++;
      // std::cout << "Invalid state encountered in motion validation.\n";
      return false;
    }

    auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
    auto deltaPos = space->distanceSpace(s1, s2);
    auto deltaT = s2->as<ob::CompoundState>()
                      ->as<ob::TimeStateSpace::StateType>(1)
                      ->position -
                  s1->as<ob::CompoundState>()
                      ->as<ob::TimeStateSpace::StateType>(1)
                      ->position;

    if (deltaT <= 0 || deltaPos / deltaT > vMax_) {
      invalid_++;
      // std::cout << "Invalid state encountered in motion validation.\n";
      return false;
    }

    auto *intermediate_state = space->allocState();

    // Interpolate points between to make sure the motion is valid
    for (float intermediate_time = 0; intermediate_time < deltaT;
         intermediate_time += SAMPLING_RESOLUTION) {
      space->interpolate(s1, s2,
                         intermediate_time +
                             s1->as<ob::CompoundState>()
                                 ->as<ob::TimeStateSpace::StateType>(1)
                                 ->position,
                         intermediate_state);

      if (!si_->isValid(intermediate_state)) {
        invalid_++;
        // std::cout << "Invalid state encountered in motion validation.\n";
        space->freeState(intermediate_state);
        return false;
      }
    }

    space->freeState(intermediate_state);

    return true;
  }

  auto checkMotion(const ob::State *, const ob::State *,
                   std::pair<ob::State *, double> &) const -> bool override {
    throw ompl::Exception(
        "checkMotion with pair not implemented for SpaceTimeMotionValidator");
  }

private:
  double vMax_;                // Maximum velocity
  ob::StateSpace *stateSpace_; // state space for distance calculations
};

void initialize_planner(int space_dim, float delta, float v_max,
                        float t_weight) {
  // Set global parameters
  SPACE_DIM = space_dim;
  DELTA = delta;
  V_MAX = v_max;
  T_WEIGHT = t_weight;
}

auto print_function_call(
    const std::array<double, 3> &current_state,
    const std::array<double, 2> &goal_state,
    const std::function<bool(double, double, double)> &is_valid) {
  std::cout << "Function call with current_state: [" << current_state[0] << ", "
            << current_state[1] << ", " << current_state[2]
            << "] and goal_state: [" << goal_state[0] << ", " << goal_state[1]
            << "]\n";
}

auto plan_strrt(const std::array<double, 3> &current_state,
                const std::array<double, 2> &goal_state,
                const std::function<bool(double, double, double)> &is_valid)
    -> std::optional<std::vector<std::array<double, 3>>> {

  // print_function_call(current_state, goal_state, is_valid);

  auto vector_space(std::make_shared<ob::RealVectorStateSpace>(2));
  auto space = std::make_shared<ob::SpaceTimeStateSpace>(vector_space, V_MAX);

  // Set bounds for R^2
  ob::RealVectorBounds spatial_bounds(2);
  spatial_bounds.setLow(0);
  spatial_bounds.setHigh(SPACE_DIM);
  vector_space->setBounds(spatial_bounds);

  double distance_between_states =
      ((goal_state[0] - current_state[0]) *
       (goal_state[0] - current_state[0])) +
      ((goal_state[1] - current_state[1]) * (goal_state[1] - current_state[1]));

  // Set time bounds for the space-time state space
  space->setTimeBounds(current_state[2],
                       current_state[2] +
                           std::max(5.0, (2.0 * distance_between_states)));

  // Create the space information class
  ob::SpaceInformationPtr space_info =
      std::make_shared<ob::SpaceInformation>(space);

  // Create and set the validity checker with dynamic obstacles
  auto validity_checker =
      std::make_shared<ValidityChecker>(space_info, is_valid);

  auto motion_validator =
      std::make_shared<SpaceTimeMotionValidator>(space_info);

  space_info->setStateValidityChecker(validity_checker);
  space_info->setMotionValidator(motion_validator);

  // Create a simple-setup
  og::SimpleSetup simple_setup(space_info);

  ob::ScopedState<> start(space);
  start[0] = current_state[0]; // x
  start[1] = current_state[1]; // y
  start[2] = current_state[2]; // t

  ob::ScopedState<> goal(space);
  goal[0] = goal_state[0]; // x
  goal[1] = goal_state[1]; // y

  // Before we begin, verify that the start and goal states are valid
  // if (!space_info->isValid(start.get()) || !space_info->isValid(goal.get()))
  // {
  //   std::cout << "Start or goal state is invalid.\n";
  //   return std::nullopt;
  // }

  simple_setup.setStartAndGoalStates(start, goal);

  // Create the planner object
  auto strrt_star = std::make_shared<og::STRRTstar>(space_info);

  // Set the range for the planner
  // strrt_star->setRange(V_MAX);

  strrt_star->setRange(std::clamp(0.1 * distance_between_states, 0.1, 5.0));

  // set the planner for the problem
  simple_setup.setPlanner(ob::PlannerPtr(strrt_star));

  auto termination_condition = ob::plannerOrTerminationCondition(
      ob::timedPlannerTerminationCondition(2.00), // 5 second timeout
      ob::CostConvergenceTerminationCondition(
          simple_setup.getProblemDefinition(), 10, 0.05));

  // auto termination_condition = ob::timedPlannerTerminationCondition(1.0);

  std::cout << "Starting planning...\n";
  ob::PlannerStatus solved = simple_setup.solve(termination_condition);

  if (!solved || solved == ob::PlannerStatus::TIMEOUT) {
    std::cout << "Planner failed to find a solution.\n";
    std::cout << "Planner status: " << solved.asString() << "\n";
    return std::nullopt;
  }

  std::cout << "Planner found a solution.\n";

  // Extract waypoints from the solution path
  const auto &path = simple_setup.getSolutionPath();
  std::vector<std::array<double, 3>> waypoints;

  for (size_t i = 0; i < path.getStateCount(); ++i) {
    const auto *state = path.getState(i);
    const auto *spatial_state =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(
            0);
    const auto *time_state =
        state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);

    std::array<double, 3> waypoint = {
        spatial_state->values[0], // x
        spatial_state->values[1], // y
        time_state->position      // t
    };
    waypoints.push_back(waypoint);
  }

  return waypoints;
  // std::vector<std::array<double, 3>> waypoints;
  // return waypoints; // For now

  /*
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

  // Verify setup before solving
  try {
    strrt_planner->setup();
  } catch (const std::exception &e) {
    std::cerr << "Planner setup failed: " << e.what() << '\n';
    return std::nullopt;
  }

  // Verify space info is properly set up
  if (!space_info->isSetup()) {
    std::cerr << "SpaceInformation not properly set up\n";
    return std::nullopt;
  }

  // Verify problem definition has start and goal states
  if (problem_def->getStartStateCount() == 0) {
    std::cerr << "No start states defined\n";
    return std::nullopt;
  }

  if (!problem_def->getGoal()) {
    std::cerr << "No goal defined\n";
    return std::nullopt;
  }

  std::cout << "Starting planning...\n";

  // Solve the planning problem with time limit
  // Combine a time limit and an optimality improvement threshold for early
  // termination
  // auto termination_condition = ob::plannerOrTerminationCondition(
  //     ob::timedPlannerTerminationCondition(5.0), // 5 second timeout
  //     ob::CostConvergenceTerminationCondition(problem_def));

  auto termination_condition = ob::timedPlannerTerminationCondition(5.0);

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
        if (!state) {
          std::cerr << "Error: Null state in path at index " << i << "\n";
          continue; // Skip this waypoint
        }

        const auto *complete_state =
            state->as<ob::SpaceTimeStateSpace::StateType>();
        if (!complete_state) {
          std::cerr << "Error: Invalid state cast at index " << i << "\n";
          continue; // Skip this waypoint
        }

        // Extract spatial and time components
        const auto *spatial_state =
            complete_state->as<ob::RealVectorStateSpace::StateType>(0);
        if (!spatial_state || !spatial_state->values) {
          std::cerr << "Error: Invalid spatial state at index " << i << "\n";
          continue; // Skip this waypoint
        }

        const auto *time_state =
            complete_state->as<ob::TimeStateSpace::StateType>(1);
        if (!time_state) {
          std::cerr << "Error: Invalid time state at index " << i << "\n";
          continue; // Skip this waypoint
        }

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
  */
}

auto geometric_path_to_serialized(const og::PathGeometric &path,
                                  int max_path_length, double *path_out,
                                  int *actual_path_length) -> void {

  size_t num_states = path.getStateCount();

  if (num_states > static_cast<size_t>(max_path_length)) {
    *actual_path_length = 0;
    std::cout << "Error: Not enough space in output array for path.\n";
    return;
  }

  // Write the path length
  *actual_path_length = static_cast<int>(num_states);

  // Copy path data to output array
  // Format: [x1, y1, t1, ...]
  for (size_t i = 0; i < num_states; i++) {
    const auto *waypoint = path.getState(i);
    const auto *spatial = waypoint->as<ob::CompoundState>()
                              ->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *time_state =
        waypoint->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1);

    path_out[(i * 3) + 0] = spatial->values[0];   // x
    path_out[(i * 3) + 1] = spatial->values[1];   // y
    path_out[(i * 3) + 2] = time_state->position; // t
  }
}
