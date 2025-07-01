module PathTracker2D

using ..R3RCommon
using ..Gatekeeper
using ..STRRT_STAR

using StaticArrays, Random, Agents, LinearAlgebra

export construct_candidate, get_position
export init_2d_path_tracker_problem

"""
    construct_candidate(agent, model)

Constructs a candidate trajectory for the agent based on its current state and the model parameters.
"""
function Gatekeeper.construct_candidate(agent::PathFollower2D, model)
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

    # Step 6: Check between t and t_bak if the agent collides with any obstacles
    # or other agents. If so, choose the last point before the collision as the backup set
    # should_replan_instead = check_for_collisions(agent, model, nominal_trajectory, t_bak)

    # if should_replan_instead
    #     println("\tAgent $(agent.id) should try to replan instead of using current noimnal.")
    #     return nothing
    # end

    # Step 5: Validate the safety of the backup set
    if !validate_backup_set(agent, model, backup_set, t_bak)
        println("\tFailed to validate backup set.")
        return nothing
    end

    if (nominal_trajectory[3, 1] - scaled_time(model) >= model.dt)
        @error "Nominal trajectory start time does not match current model time. Expected $(scaled_time(model)), got $(nominal_trajectory[3, 1])"
    end

    # Create a candidate trajectory
    candidate_trajectory = PathFollowerCompositeTrajectory(
        scaled_time(model),
        t_bak, # backup time = switch time
        t_bak,
        nominal_trajectory,
        backup_set
    )

    return candidate_trajectory
end

"""
    construct_new_nominal(agent, model)
Constructs a new nominal given agents position and goal using OMPL.
"""
function construct_new_nominal(agent::PathFollower2D, model)
    neighbor_agent_trajectories::Vector{PathFollowerCompositeTrajectory} = [neighbor.committed_trajectory for neighbor in comms_radius(agent, model)]
    stamped_start_pos = @SVector [agent.pos[1], agent.pos[2], scaled_time(model)]
    return plan_nominal(stamped_start_pos, agent.goal, make_valid_function(neighbor_agent_trajectories, model))
end

"""
    choose_backup_set(agent, model, nominal_trajectory)
Selects a backup set for the agent based on the nominal trajectory.

Returns the backup_set and time t_bak.
"""
function choose_ideal_backup_set(agent::PathFollower2D, model, nominal_trajectory)
    # Find the first point in the nominal trajectory that is outside the planning radius and not been visited yet
    current_time = scaled_time(model)
    exit_idx = findfirst(x -> x[3] >= current_time && squared_dist(x[1:2], agent.pos) > model.Rplan^2, eachcol(nominal_trajectory))

    # If the nominal is entirely within the planning radius,
    # return the last point on the nominal
    if (exit_idx === nothing)
        return nominal_trajectory[1:2, end], nominal_trajectory[3, end]
    end

    if exit_idx == 1
        # something is wrong
        @error "Nominal trajectory exits planning radius at the first point, which should not happen."
        return nothing
    end

    # If not, somewhere between exit_idx and exit_idx - 1 is the last point in the planning radius
    intersection_point = compute_intersection_point(
        nominal_trajectory[1:2, exit_idx-1],
        nominal_trajectory[1:2, exit_idx],
        agent.pos,
        model.Rplan
    )

    if intersection_point === nothing
        @error "Failed to compute intersection point for backup set."
        return nothing
    end

    intersection_time = nominal_trajectory[:, exit_idx-1][3] +
                        norm(nominal_trajectory[1:2, exit_idx-1] - intersection_point)

    # TODO REMOVE
    @assert isa(intersection_point, AbstractVector) "Backup set must be an AbstractVector, got $(typeof(backup_set))"

    return intersection_point, intersection_time
end

"""
    validate_backup_set(agent, model, backup_set, t_bak)
Validates the backup set against the agent's neighbors to ensure no collisions.

If two backup sets overlap, it returns false. Also verifies that no agents's committed trajectories
intersect with the backup set before they enter their own backup sets.
"""
function validate_backup_set(agent::PathFollower2D, model, backup_set, t_bak)
    current_neighbors = comms_radius(agent, model)

    # TODO REMOVE
    @assert isa(backup_set, AbstractVector) "Backup set must be an AbstractVector, got $(typeof(backup_set))"

    for neighbor in current_neighbors
        # If backup sets overlap, no good.
        if squared_dist(backup_set[1:2], neighbor.committed_trajectory.backup_set) < model.delta^2
            return false
        end

        if neighbor.committed_trajectory.t_bak >= t_bak
            continue
        end

        idx = findlast(x -> x[3] < t_bak, eachcol(neighbor.committed_trajectory.nominal_trajectory))

        if idx === nothing || idx === size(neighbor.committed_trajectory.nominal_trajectory, 2)
            @error "No point on the nominal of neighbor less than t_bak but their backup less than agent's? Should never happen"
            continue
        end

        # Loop through every pair of points in the neighbor's nominal trajectory,
        # starting with (idx, idx+1) until no more pairs exist, checking for intersections
        # with the backup set.
        length_of_neighbor_nominal = size(neighbor.committed_trajectory.nominal_trajectory, 2)
        for i in idx:(length_of_neighbor_nominal-1)
            if compute_intersection_point(
                neighbor.committed_trajectory.nominal_trajectory[1:2, i],
                neighbor.committed_trajectory.nominal_trajectory[1:2, i+1],
                backup_set[1:2],
                model.delta
            ) !== nothing
                return false
            end
        end
    end

    return true
end

function Gatekeeper.get_position(trajectory::PathFollowerCompositeTrajectory, t::Float64)
    if t >= trajectory.t_bak
        return trajectory.backup_set[1:2]
    else
        # Interpolate along the nominal trajectory
        idx = findlast(x -> x[3] <= t, eachcol(trajectory.nominal_trajectory))

        if idx === nothing || idx == size(trajectory.nominal_trajectory, 2)
            # @error "No point on the nominal trajectory less than current time, or at the end of the trajectory. Should never happen."
            # @show "Candidate time: $t"
            # @show "Backup Time: $(trajectory.t_bak)"
            # @show "end time: $(trajectory.nominal_trajectory[3, end])"
            # @show "start time:  $(trajectory.nominal_trajectory[3, 1])"
            # @show idx
            # @show trajectory.nominal_trajectory
            return trajectory.nominal_trajectory[1:2, end]  # Return the last point if no valid idx found
        end

        p1 = trajectory.nominal_trajectory[:, idx]
        p2 = trajectory.nominal_trajectory[:, idx+1]

        t_ratio = (t - p1[3]) / (p2[3] - p1[3])
        return p1[1:2] + t_ratio * (p2[1:2] - p1[1:2])
    end
end

"""
    propagate_along_trajectory!(agent, model)
Propagates the agent along its committed trajectory, either to the backup set or along the nominal.
"""
function Gatekeeper.propagate_along_trajectory!(agent::PathFollower2D, model)
    move_agent!(agent, get_position(agent.committed_trajectory, scaled_time(model)), model)
end


function init_2d_path_tracker_problem(;
    n_agents::Int=10, ## Number of agents
    delta::Float64=1.0, ## Inter-agent collision radius
    Rcomm::Float64=16.0, ## Communication Radius
    Rgoal::Float64=0.5, ## Goal Radius 
    dt::Float64=0.1, ## Time Step
    seed::Int=1234, ## Random Seed
    starting_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing, ## Starting positions of agents
    goal_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing ## Goal positions of agents
)
    dims = (100, 100)
    dims_vector = @SVector [dims[1], dims[2]]

    Random.seed!(seed)

    # If starting agents positions/goals not provided, generate random ones
    if starting_positions === nothing
        starting_positions = [SVector{2,Float64}(rand(2)) .* dims_vector for _ in 1:n_agents]
    end
    if goal_positions === nothing
        goal_positions = [SVector{2,Float64}(rand(2)) .* dims_vector for _ in 1:n_agents]
    end

    v0 = @SVector [0.0, 0.0] # Initial velocity of agents

    # Create the agents vector

    function add_agents!(model)
        for (starting_position, goal_position) in zip(starting_positions, goal_positions)
            agent = PathFollower2D(model, starting_position, v0, false, false, false, goal_position, nothing)
            add_agent_own_pos!(agent, model)
        end
    end

    return init_model(
        PathFollower2D, # Type of agent to create
        add_agents!, # vector of agents of correct type
        nothing; # no model specific parameters
        delta=delta,
        Rcomm=Rcomm,
        Rgoal=Rgoal,
        dt=dt,
        rng=MersenneTwister(seed),
        dims=dims
    )
end

end