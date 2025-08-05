module DubinsAgent2DModel

using ..R3RCommon
using ..Gatekeeper

using DynamicRRT, Dubins
using StaticArrays, Random, Agents, LinearAlgebra

export construct_candidate, get_position
export init_dubins_agent_2d_problem

function Gatekeeper.construct_candidate(agent::DubinsAgent2D, model)
    # Get the nominal to track
    if (!agent.in_network || agent.failed_last_replan || scaled_time(model) >= agent.committed_trajectory.t_switch - model.num_timesteps_before_replan * model.dt) || (scaled_time(model) - agent.committed_trajectory.t_committed >= 10.0)
        # Needs a new nominal
        nominal_trajectory = construct_new_nominal(agent, model)
    else
        # Can continue to track the existing nominal
        nominal_trajectory = agent.committed_trajectory.nominal_trajectory
    end

    # Validate the nominal trajectory exists
    if nominal_trajectory === nothing
        println("\tAgent $(agent.id) failed to construct a nominal trajectory.")
        return nothing
    end

    # Choose a backup set -- either where the nominal leaves the planning radius, 
    # or immediately before it intersects with an obstacle
    backup_result = choose_ideal_backup_set(agent, model, nominal_trajectory)
    if backup_result === nothing
        @error "Failed to choose a backup set for agent $(agent.id)."
        return nothing
    end
    backup_set, t_bak = backup_result

    # Step 5: Validate the safety of the backup set
    if !validate_backup_set(agent, model, backup_set, t_bak)
        println("\tFailed to validate backup set.")
        return nothing
    end
    if (nominal_trajectory[3, 1] - scaled_time(model) >= model.dt)
        @error "Nominal trajectory start time does not match current model time. Expected $(scaled_time(model)), got $(nominal_trajectory[3, 1])"
    end

    # Create a candidate trajectory
    candidate_trajectory = DubinsCompositeTrajectory(
        scaled_time(model),
        t_bak, # backup time = switch time
        t_bak,
        nominal_trajectory,
        backup_set
    )

    return candidate_trajectory
end

function construct_new_nominal(agent::DubinsAgent2D, model)::Union{Nothing,Vector{Tuple{DubinsPath,F<:Real}}}
    sol = plan_nominal_with_rrt(agent, model)

    if sol.status != RRTStar.SolutionStatus.GoalReached
        return nothing # Failed to find a solution
    end

    # Convert the solution from the solver into a vector of DubinsPath objects
    solution_path = Vector{DubinsPath}(undef, length(sol.best_path) - 1)
    for i in SOneTo(length(sol.best_path) - 1)
        errcode, path = dubins_shortest_path(
            sol.best_path[i],
            sol.best_path[i+1],
            agent.turning_radius
        )

        if errcode != EDUBOK
            @error "Dubins path error: $errcode"
            return nothing
        end

        solution_path[i] = path
    end

    cumulative_lengths = cumsum([scaled_time(model); map(dubins_path_length, solution_path)])
    return collect(zip(solution_path, cumulative_lengths)) # Vector of tuples (DubinsPath, cumulative_length)
end

function plan_nominal_with_rrt(agent::DubinsAgent2D, model)::DynamicRRT.RRTStarSolution
    domain_min = @SVector [0.0, 0.0, -π]
    domain_max = @SVector [model.dims[1], model.dims[2], π]
    rrt_domain = (domain_min, domain_max)

    # Convert the obstacles to the format required by the RRT* planner
    static_obstacles = model.obstacles

    static_obstacles = [
        CircleObstacle(ob.center, ob.radius)
        for ob in static_obstacles
    ]

    neighbor_agent_trajectories::Vector{DubinsCompositeTrajectory} = [neighbor.committed_trajectory for neighbor in comms_radius(agent, model)]
    dynamic_obstacles = [
        DubinsDynamicPathRRT.DynamicCircleObstacle(
            t -> Gatekeeper.get_position(traj, t)[SOneTo(2)],
            model.delta,
            (scaled_time(model), Inf)
        )
        for traj in neighbor_agent_trajectories if traj !== nothing
    ]

    # Create the RRT* planner
    rrt_problem = DubinsDynamicPathRRT.DubinsDynamicRRTProblem(
        rrt_domain,
        agent.turning_radius,
        static_obstacles,
        dynamic_obstacles,
        agent.goal # TODO Verify goal type is compatible here and works with rest of the code
    )

    # Convert the start state of the agent to the required format
    start_state = @SVector [agent.pos[1], agent.pos[2], agent.theta]
    widths = @SVector [0.01, 0.01]
    sol = RRTStar.setup(rrt_problem, start_state, widths)

    RRTStar.solve!(
        rrt_problem, sol;
        max_iterations=1000,
        max_time_seconds=1.0,
        do_rewire=true,
        early_exit=true,
        goal_bias=0.1
    )

    return sol
end

function choose_ideal_backup_set(agent::DubinsAgent2D, model, nominal_trajectory)::Tuple{SVector{3,F<:Real},F<:Real}
    # Find the first point in the nominal trajectory that is outside the planning radius and not been visited yet
    current_time = scaled_time(model)

    # idx of the first path that:
    # 1. Starts at or after the current time
    # 2. Starts outside the planning radius
    # Thus, the previous path must end at or after the current time
    # and end outside the planning radius
    exit_idx = findfirst((path, start_time) -> start_time >= current_time && squared_dist(path.qi[SOneTo(2)], agent.pos) > model.Rplan^2, nominal_trajectory)

    # If the nominal is entirely within the planning radius,
    # return the last point on the nominal
    if (exit_idx === nothing)
        last_path, last_path_start_time = nominal_trajectory[end]
        last_path_length = dubins_path_length(last_path)
        last_path_end_time = last_path_start_time + last_path_length
        last_path_end_point = dubins_path_sample(last_path, last_path_length)

        return last_path_end_point, last_path_end_time
    end

    # Idx of the first path that:
    # 1. Ends at or after the current time
    # 2. Ends outside the planning radius
    exit_idx -= 1 # Get the idx matching the second set of conditions
    @assert exit_idx > 0 "Exit index must be greater than 0, but got $exit_idx"

    # Find the first time along the dubins path that is outside the planning radius
    exit_point, exit_time = compute_exit_point(
        agent,
        model,
        nominal_trajectory[exit_idx][1] # path component of the tuple
    )

    return exit_point, exit_time
end

function compute_exit_point(agent::DubinsAgent2D, model, dubins_path::DubinsPath; sample_resolution::Float64=0.01)::Tuple{SVector{3,F<:Real},F<:Real}
    # Find the first time along the dubins path that is outside the planning radius
    path_len = dubins_path_length(dubins_path)

    # Ensure the sample resolution is within a valid range
    @assert 0 < sample_resolution <= 1.0 "Sample resolution must be in (0, 1], but got $sample_resolution"

    for t in sample_resolution:sample_resolution:1.0
        point = dubins_path.sample(t * path_len)

        # If current point is outside the planning radius, return the previously sampled point
        if squared_dist(point[SOneTo(2)], agent.pos) > model.Rplan^2
            return dubins_path.sample((t - sample_resolution) * path_len), t - sample_resolution
        end
    end

    # If we reach here, the entire path is within the planning radius (?) so return the last?
    @assert false "Dubins path did not exit the planning radius, which should not happen."
    return dubins_path.end_point, dubins_path.end_time
end


# TODO -- WORK WITH TRULY PERIODIC BACKUP SETS
function validate_backup_set(agent::DubinsAgent2D, model, backup_set, t_bak; sampling_resolution::Float64=0.01)::Bool
    current_neighbors = comms_radius(agent, model)

    for neighbor in current_neighbors

        # If backup sets are too close, return false
        if squared_dist(backup_set[SOneTo(2)], neighbor.committed_trajectory.backup_set[SOneTo(2)]) < (model.delta)^2
            return false
        end

        # If neighbor enters backup set before agent, then for neighbor to collide w/agent,
        # would be caught by above check, e.g. by time agent is in backup, so will neighbor
        if t_bak >= neighbor.committed_trajectory.t_bak
            continue
        end

        # index of first path which has points at times after t_bak
        # e.g. entirety of next path is after t_bak (could also be last path)
        idx = findlast((path, start_time) -> start_time < t_bak, neighbor.committed_trajectory.nominal_trajectory)

        if idx === nothing
            @error "Failed to find valid idx in neighbor nominal trajectory for agent $(agent.id) and neighbor $(neighbor.id)."
            return false
        end

        # For all rest of paths in neighbor nominal, starting from idx until neighbor enters its backup
        # check if they collide with the agent's backup set
        for (path, _) in neighbor.committed_trajectory.nominal_trajectory[idx:end]
            path_length = dubins_path_length(path) # length of current path
            xy_back = backup_set[SOneTo(2)] # xy coordinate of backup

            # Check if any point along the path is within delta of the backup set
            if any(t -> squared_dist(dubins_path_sample(path, t * path_length), xy_back) < (model.delta)^2, 0:sampling_resolution:1.0)
                return false
            end
        end
    end

    return true
end

function Gatekeeper.get_position(trajectory::DubinsCompositeTrajectory, t::Float64)::SVector{3,F<:Real}
    # Handle backup set case
    # TODO parametric backup sets
    if t >= trajectory.t_bak
        return trajectory.backup_set#[SOneTo(3)] # TODO Verify I am being consistent with 3D state
    end

    # Must be in nominal trajectory
    idx = findlast((path, start_time) -> start_time <= t, trajectory.nominal_trajectory)
    if idx === nothing
        last_trajectory = trajectory.nominal_trajectory[end][1]
        return dubins_path_sample(last_trajectory, dubins_path_length(last_trajectory))
    end

    # Difference in path start time and current time
    time_along_path = t - trajectory.nominal_trajectory[idx][2]

    return dubins_path_sample(trajectory.nominal_trajectory[idx][1], time_along_path)
end

function Gatekeeper.propagate_along_trajectory!(agent::DubinsAgent2D, model)
    new_position = get_position(agent.committed_trajectory, scaled_time(model))

    agent.theta = new_position[3] # Update the agent's orientation
    move_agent!(agent, new_position[SOneTo(2)], model) # Update agent's xy posiiton
end

function init_dubins_agent_2d_problem(;
    n_agents::Int=10, ## Number of agents
    delta::Float64=0.05, ## Inter-agent collision radius
    Rcomm::Float64=0.3, ## Communication Radius
    Rgoal::Float64=0.05, ## Goal Radius 
    dt::Float64=0.005, ## Time Step
    seed::Int=1234, ## Random Seed
    turning_radius::Float64=0.05, ## Minimum turning radius for Dubins dynamics,
    starting_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing, ## Starting positions of agents
    goal_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing ## Goal positions of agents
)
    dims = (1.0, 1.0)

    dims_vector = @SVector [dims[1], dims[2]]
    dims_min = @SVector [0.0, 0.0, -π]
    dims_max = @SVector [dims[1], dims[2], π]

    dims_diff = dims_max - dims_min


    Random.seed!(seed)

    if starting_positions === nothing
        starting_positions = [SVector{3,Float64}(rand(3)) .* dims_diff + dims_min for _ in SOneTo(n_agents)]
    end
    if goal_positions === nothing
        goal_positions = [SVector{3,Float64}(rand(3)) .* dims_diff + dims_min for _ in SOneTo(n_agents)]
    end

    v0 = @SVector [0.0, 0.0] # Initial velocity of agents -- not used in this at all so like (?)

    function add_agents!(model)
        for (starting_position, goal_position) in zip(starting_positions, goal_positions)
            agent = DubinsAgent2D(model, starting_position[SOneTo(2)], v0, starting_position[3], false, false, false, goal_position, nothing, turning_radius)
            add_agent_own_pos!(agent, model)
        end
    end

    return init_model(
        DubinsAgent2D,
        add_agents!, # vector of agents of correct type
        nothing; # no model specific parameters
        delta=delta,
        Rcomm=Rcomm,
        Rgoal=Rgoal,
        dt=dt,
        rng=MersenneTwister(seed),
        dims=dims)
end

end