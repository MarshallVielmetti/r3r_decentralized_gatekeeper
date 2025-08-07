module DubinsAgent2DModel

using ..R3RCommon
using ..Gatekeeper

using DynamicRRT, Dubins
using StaticArrays, Random, Agents, LinearAlgebra

export construct_candidate, get_position
export init_dubins_agent_2d_problem

function Gatekeeper.construct_candidate(agent::DubinsAgent2D, model)
    # println("[TRACE] Entered Gatekeeper.construct_candidate")
    # Get the nominal to track
    if (!agent.in_network || agent.failed_last_replan || scaled_time(model) >= agent.committed_trajectory.t_switch - model.num_timesteps_before_replan * model.dt) || (scaled_time(model) - agent.committed_trajectory.t_committed >= 10.0)
        # Needs a new nominal
        nominal_trajectory = construct_new_nominal(agent, model)
    else
        # Can continue to track the existing nominal
        nominal_trajectory = agent.committed_trajectory.nominal_trajectory
    end

    # Validate the nominal trajectory exists
    if nominal_trajectory === nothing || length(nominal_trajectory) == 0
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

    if (nominal_trajectory[1][2] - scaled_time(model) >= model.dt)
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

function construct_new_nominal(agent::DubinsAgent2D, model)::Union{Nothing,Vector{Tuple{DubinsPath,Float64}}}
    # println("[TRACE] Entered construct_new_nominal")
    sol = plan_nominal_with_rrt(agent, model)

    if sol.status != RRTStar.GoalReachable
        @error "RRT* failed to find a solution for agent $(agent.id). Status: $(sol.status)"
        return nothing

        # Fallback: Direct Dubins path to goal
        # errcode, path = dubins_shortest_path(
        #     SVector{3,Float64}(agent.pos[1], agent.pos[2], agent.theta),
        #     agent.goal,
        #     agent.turning_radius
        # )
        # return [(path, scaled_time(model))]
    end

    if isnothing(sol.best_path) || length(sol.best_path) < 2
        @error "RRT* solution path too short for agent $(agent.id). Length: $(length(sol.best_path)). Status: $(sol.status)"
        return nothing
    end

    # Convert the solution from the solver into a vector of DubinsPath objects
    solution_path = Vector{DubinsPath}(undef, length(sol.best_path) - 1)
    for i in SOneTo(length(sol.best_path) - 1)
        errcode, path = dubins_shortest_path(
            sol.best_path[i][SOneTo(3)],
            sol.best_path[i+1][SOneTo(3)],
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

function plan_nominal_with_rrt(agent::DubinsAgent2D, model)::RRTStar.RRTStarSolution
    # println("[TRACE] Entered plan_nominal_with_rrt")
    # dims = @SVector [0.98, 0.98] # TODO Actually adapt but its being a pos

    dims = model.dims
    padding = @SVector [0.1, 0.1, 0.0]

    domain_min = @SVector [0.0 + padding[1], 0.0 + padding[2], -π]
    domain_max = @SVector [dims[1] - padding[1], dims[2] - padding[2], π]
    rrt_domain = (domain_min, domain_max)

    # Convert the obstacles to the format required by the RRT* planner
    # TODO Add the static obstacles in
    # static_obstacles = model.obstacles

    # static_obstacles = [
    #     CircleObstacle(ob.center, ob.radius)
    #     for ob in static_obstacles
    # ]
    static_obstacles = []

    neighbor_agent_trajectories::Vector{DubinsCompositeTrajectory} = [neighbor.committed_trajectory for neighbor in comms_radius(agent, model)]
    dynamic_obstacles = [
        DubinsDynamicPathRRT.DynamicCircleObstacle(
            t -> Gatekeeper.get_position(traj, t),
            model.delta,
            (scaled_time(model), Inf)
        )
        for traj in neighbor_agent_trajectories if traj !== nothing
    ]

    # TODO Verify this results in better performance
    # includes the entirety of an agents nominal as an obstacle for other agents to avoid
    # while planning
    rest_of_nominals = [
        DubinsDynamicPathRRT.DynamicCircleObstacle(
            t -> Gatekeeper.get_nominal_position(traj, t),
            model.delta,
            (traj.t_bak, Inf)
        )
        for traj in neighbor_agent_trajectories if traj !== nothing && traj.t_bak > scaled_time(model)
    ]

    dynamic_obstacles = vcat(dynamic_obstacles, rest_of_nominals)

    # Create the RRT* planner
    rrt_problem = DubinsDynamicPathRRT.DubinsDynamicRRTProblem(
        rrt_domain,
        agent.turning_radius,
        static_obstacles,
        dynamic_obstacles,
        agent.goal, # TODO Verify goal type is compatible here and works with rest of the code
        true # enable strict bounds checking
    )

    # Convert the start state of the agent to the required format
    start_state = @SVector [agent.pos[1], agent.pos[2], agent.theta, scaled_time(model)] # x, y, theta, time
    widths = @SVector [0.01, 0.01]
    sol = RRTStar.setup(rrt_problem, start_state, widths)

    RRTStar.solve!(
        rrt_problem, sol;
        max_iterations=50,
        max_time_seconds=1.0,
        do_rewire=true,
        early_exit=true, # TODO make better exit metric
        goal_bias=0.1
    )

    return sol
end

function choose_ideal_backup_set(agent::DubinsAgent2D, model, nominal_trajectory)::Tuple{SVector{3,Float64},Float64}
    # println("[TRACE] Entered choose_ideal_backup_set")
    # Find the first point in the nominal trajectory that is outside the planning radius and not been visited yet
    current_time = scaled_time(model)

    # idx of the first path that:
    # 1. Starts at or after the current time
    # 2. Starts outside the planning radius
    # Thus, the previous path must end at or after the current time
    # and end outside the planning radius
    exit_idx = findlast(nominal_trajectory) do (path, start_time)
        path_end_time = start_time + dubins_path_length(path)
        path_end_time >= current_time && squared_dist(path.qi[SOneTo(2)], agent.pos) <= model.Rplan^2
    end

    # No path on the nominal is in the future at all and in the planning radius?
    if (exit_idx === nothing)
        @assert false "This should never happen???"

        last_path, last_path_start_time = nominal_trajectory[end]
        last_path_length = dubins_path_length(last_path)
        last_path_end_time = last_path_start_time + last_path_length
        _, last_path_end_point = dubins_path_sample(last_path, last_path_length)

        return last_path_end_point, last_path_end_time
    end


    # Idx of the first path that:
    # 1. Ends at or after the current time
    # 2. Ends outside the planning radius
    # exit_idx -= 1 # Get the idx matching the second set of conditions
    # @assert exit_idx > 0 "Exit index must be greater than 0, but got $exit_idx"

    # Find the first time along the dubins path that is outside the planning radius
    exit_point, exit_time = compute_exit_point(
        agent,
        model,
        nominal_trajectory[exit_idx][1] # path component of the tuple
    )
    exit_time += nominal_trajectory[exit_idx][2] # Add the start time of the path

    return exit_point, exit_time
end

function compute_exit_point(agent::DubinsAgent2D, model, dubins_path::DubinsPath; sample_resolution::Float64=0.01)::Tuple{SVector{3,Float64},Float64}
    # println("[TRACE] Entered compute_exit_point")
    # Find the first time along the dubins path that is outside the planning radius
    path_len = dubins_path_length(dubins_path)

    # Ensure the sample resolution is within a valid range
    @assert 0 < sample_resolution <= 1.0 "Sample resolution must be in (0, 1], but got $sample_resolution"

    for t in sample_resolution:sample_resolution:1.0
        _, point = dubins_path_sample(dubins_path, t * path_len)

        # If current point is outside the planning radius, return the previously sampled point
        if squared_dist(point[SOneTo(2)], agent.pos) > model.Rplan^2
            len_along_path = (t - sample_resolution) * path_len
            _, exit_point = dubins_path_sample(dubins_path, len_along_path)
            return exit_point, len_along_path
        end
    end

    # If we reach here, the entire path is within the planning radius (?) so return the last?
    # @assert false "Dubins path did not exit the planning radius, which should not happen."
    return dubins_path_sample(dubins_path, path_len)[2], path_len
end


# TODO -- WORK WITH TRULY PERIODIC BACKUP SETS
function validate_backup_set(agent::DubinsAgent2D, model, backup_set, t_bak; sampling_resolution::Float64=0.02)::Bool
    # println("[TRACE] Entered validate_backup_set")
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
        idx = findlast(tup -> tup[2] < t_bak, neighbor.committed_trajectory.nominal_trajectory)

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
            if any(t -> squared_dist(dubins_path_sample(path, t * path_length)[2][SOneTo(2)], xy_back) < (model.delta)^2, 0:sampling_resolution:1.0)
                return false
            end
        end
    end

    return true
end

function Gatekeeper.get_position(trajectory::DubinsCompositeTrajectory, t::Float64)::SVector{2,Float64}
    # println("[TRACE] Entered Gatekeeper.get_position")
    return get_pose(trajectory, t)[SOneTo(2)]
end

function Gatekeeper.get_nominal_position(trajectory::DubinsCompositeTrajectory, t::Float64)::SVector{2,Float64}
    if t < trajectory.t_committed
        return trajectory.nominal_trajectory[1].qi[SOneTo(2)]
    end

    total_trajectory_length = sum(dubins_path_length(path) for (path, _) in trajectory.nominal_trajectory)

    # If time is greater than the end of the nominal trajectory, return the end of the last path
    if t >= trajectory.t_committed + total_trajectory_length
        return dubins_path_sample(trajectory.nominal_trajectory[end][1], dubins_path_length(trajectory.nominal_trajectory[end][1]))[2][SOneTo(2)]
    end

    # Otherwise, return position along the nominal in the normal way
    idx = findlast(x -> begin
            path, start_time = x
            start_time <= t
        end, trajectory.nominal_trajectory)

    time_along_path = t - trajectory.nominal_trajectory[idx][2]
    return dubins_path_sample(trajectory.nominal_trajectory[idx][1], time_along_path)[2][SOneTo(2)]
end

function get_pose(trajectory::DubinsCompositeTrajectory, t::Float64)::SVector{3,Float64}
    # println("[TRACE] Entered get_pose")
    # Handle backup set case
    if t >= trajectory.t_bak
        return trajectory.backup_set
    end

    # Must be in nominal trajectory
    idx = findlast(x -> begin
            path, start_time = x
            start_time <= t
        end, trajectory.nominal_trajectory)

    if idx === nothing
        last_trajectory = trajectory.nominal_trajectory[end][1]
        return dubins_path_sample(last_trajectory, dubins_path_length(last_trajectory))[2]
    end

    # Difference in path start time and current time
    time_along_path = t - trajectory.nominal_trajectory[idx][2]
    return dubins_path_sample(trajectory.nominal_trajectory[idx][1], time_along_path)[2]
end

function Gatekeeper.propagate_along_trajectory!(agent::DubinsAgent2D, model)
    # println("[TRACE] Entered Gatekeeper.propagate_along_trajectory!")
    new_position = get_pose(agent.committed_trajectory, scaled_time(model))

    agent.theta = new_position[3] # Update the agent's orientation
    move_agent!(agent, new_position[SOneTo(2)], model) # Update agent's xy posiiton
end

function init_dubins_agent_2d_problem(;
    n_agents::Int=10, ## Number of agents
    delta::Float64=0.05, ## Inter-agent collision radius
    Rcomm::Float64=0.3, ## Communication Radius
    Rgoal::Float64=0.01, ## Goal Radius 
    dt::Float64=0.005, ## Time Step
    seed::Int=1234, ## Random Seed
    dim::Float64=1.0, ## Dimension of the world
    turning_radius::Float64=0.05, ## Minimum turning radius for Dubins dynamics,
    starting_positions::Union{Nothing,Vector{SVector{3,Float64}}}=nothing, ## Starting positions of agents
    goal_positions::Union{Nothing,Vector{SVector{3,Float64}}}=nothing ## Goal positions of agents
)
    # println("[TRACE] Entered init_dubins_agent_2d_problem")
    dims = (dim, dim)

    dims_min = @SVector [0.0, 0.0, -π]
    dims_max = @SVector [dims[1], dims[2], π]

    padding = @SVector [2 * turning_radius, 2 * turning_radius, 0.0]

    dims_diff = (dims_max - dims_min) - (2 * padding)

    Random.seed!(seed)

    if starting_positions === nothing
        starting_positions = [SVector{3,Float64}(rand(3)) .* dims_diff + dims_min + padding for _ in SOneTo(n_agents)]
    end
    if goal_positions === nothing
        goal_positions = [SVector{3,Float64}(rand(3)) .* dims_diff + dims_min + padding for _ in SOneTo(n_agents)]
    end

    for (start_pos, end_pos) in zip(starting_positions, goal_positions)
        println("Agent starting at $(start_pos) with goal $(end_pos)")
    end

    v0 = @SVector [0.0, 0.0] # Initial velocity of agents -- not used in this at all so like (?)

    function add_agents!(model)
        for (starting_position, goal_position) in zip(starting_positions, goal_positions)
            agent = DubinsAgent2D(model, starting_position[SOneTo(2)], v0, starting_position[3], false, false, false, goal_position, nothing, turning_radius, 0.0)
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