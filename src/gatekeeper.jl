"""
Gatekeeper Module

Shared functionality that works independent of the specific agent type.
"""
module Gatekeeper

using ..R3RCommon
using ..STRRT_STAR

using Agents, Random, StaticArrays

export construct_candidate, get_position, propagate_along_trajectory!

# export get_position

export init_model
export make_valid_function

@kwdef mutable struct GatekeeperParameters{TP}
    properties::TP = nothing# Specific parameters
    Rcomm::Float64 = 16.0 # Communication Radius 
    Rplan::Float64 = 5.0 # Planning Radius
    delta::Float64 = 1.0 # Inter-agent collision radius
    Rgoal::Float64 = 0.5 # Goal pose radius to prevent weird numerical instabilities
    num_timesteps_before_replan::Int = 2 # Number of timesteps before agent reaches its switch time before replanning
    dt::Float64 = 0.01 # Simulation Time Step
end

function init_model(
    agent_type::Any,
    add_agents!::Function,
    model_specific_params;
    delta::Float64=1.0,
    Rcomm::Float64=16.0,
    Rgoal::Float64=0.5,
    dt::Float64=0.1,
    rng=Random.MersenneTwister(1234),
    dims::Tuple{Int,Int}=(100, 100) # Dimensions of the
)
    println("Initializing the model!")
    Rplan = (Rcomm - delta) / 3.0

    properties = GatekeeperParameters(
        properties=model_specific_params,
        Rcomm=Rcomm,
        Rplan=Rplan,
        delta=delta,
        Rgoal=Rgoal,
        dt=dt
    )

    STRRT_STAR.init_planner(100, Float32(delta), Float32(1.0), Float32(1.0))


    # Create the model object
    model = StandardABM(agent_type, ContinuousSpace(dims; periodic=false); (agent_step!)=agent_step!, (model_step!)=model_step!, rng, properties)
    add_agents!(model)

    # Return the model
    return model
end

function model_step!(model)
    println("\n###################################")
    println("Model step at time $(scaled_time(model))")

    # Clear Replanning Flags
    for agent in allagents(model)
        agent.just_replanned = false
    end
end

function agent_step!(agent, model)
    println("Agent $(agent.id) step at time $(scaled_time(model))")

    if !agent.in_network
        if try_to_join_network!(agent, model)
            println("\tAgent $(agent.id) has joined the network successfully!")
            propagate_along_trajectory!(agent, model)
        end
        return
    end

    if in_collision(agent, model)
        @error "Agent $(agent.id) is in collision at time $(scaled_time(model))!"
    end

    if squared_dist(agent.pos, agent.goal) < model.delta^2
        # Agent has reached its goal
        println("\tAgent $(agent.id) has reached its goal at time $(scaled_time(model))!")
        remove_agent!(agent, model)
        return
    end

    if agent_should_replan(agent, model)
        println("\tAgent $(agent.id) is replanning at time $(scaled_time(model))!")
        candidate_trajectory = construct_candidate(agent, model)
        if candidate_trajectory !== nothing && validate_candidate(candidate_trajectory, agent, model)
            println("\tAgent $(agent.id) has replanned successfully.")
            update_committed_trajectory!(agent, model, candidate_trajectory)
            agent.failed_last_replan = false
        else
            printstyled("\tAgent $(agent.id) failed to replan at time $(scaled_time(model))!\n", color=:red)
            agent.failed_last_replan = true
        end
    end

    propagate_along_trajectory!(agent, model)
end

function try_to_join_network!(agent, model)
    ## Rules to Join Network
    # 1. The agent's 'join' position must not be within the collision radius of any other agent / in an obstacle
    # 2. The agent must be able to "replan" -- no "neighbors" have replanned in this iteration
    # 2. The agent must be able to construct a valid candidate trajectory that does not collide with
    #    the committed trajectories of other agents within its 3R + delta communication radius

    println("Agent $(agent.id) attempting to join the network at time $(scaled_time(model))")

    if in_collision(agent, model)
        println("\tAgent $(agent.id) can not join the network because it is in collision!")
        return false
    end

    neighbor_replanned = any(neighbor.just_replanned for neighbor in comms_radius(agent, model))
    if neighbor_replanned
        println("\tAgent $(agent.id) can not join the network because a neighbor has replanned!")
        return false
    end

    candidate_trajectory = construct_candidate(agent, model)
    if candidate_trajectory === nothing
        println("\tAgent $(agent.id) can not join the network because it could not construct a candidate trajectory!")
        return false
    end

    if !validate_candidate(candidate_trajectory, agent, model)
        println("\tAgent $(agent.id) can not join the network because it could not construct a valid candidate trajectory!")
        return false
    end

    # If we reach here, the agent can join the network!
    update_committed_trajectory!(agent, model, candidate_trajectory)
    agent.in_network = true
    # println("\tAgent $(agent.id) successfully joined the network with a new committed trajectory!")
    return true
end


function agent_should_replan(agent, model)::Bool
    ## An agent should replan if:
    # 1. It wants to replan based on some heuristic (i.e. almost at switch time)
    # 2. None of its neighbors have replanned in this iteration
    # 3. Its backup set is not ontop of its goal position

    # check if the agent even cares about replanning based on some heuristic
    wants_to_replan = scaled_time(model) > agent.committed_trajectory.t_switch - model.num_timesteps_before_replan * model.dt

    @show agent.committed_trajectory.backup_set
    if !isa(agent.committed_trajectory.backup_set, AbstractVector)
        @show agent.committed_trajectory
        @show agent
    end

    ends_at_goal = squared_dist(agent.committed_trajectory.backup_set[1:2], agent.goal) <= model.Rgoal^2
    any_neighbors_replanned = any(neighbor.just_replanned for neighbor in comms_radius(agent, model))

    # println("Wnats to Replan: $(wants_to_replan) -- switch time is $(agent.committed_trajectory.t_switch)")
    # println("Trajectory ends at goal: $(trajectory_ends_at_goal)")
    # println("My neighbor replanned: $(agent.my_neighbor_replanned)")

    return wants_to_replan && !ends_at_goal && !any_neighbors_replanned
end

function update_committed_trajectory!(agent, model, candidate::AbstractCompositeTrajectory)
    agent.just_replanned = true
    agent.committed_trajectory = candidate
end


function construct_candidate(agent, model)
    throw(ErrorException("construct_candidate not implemented for $(typeof(agent))"))
end

function get_position(agent, t::Float64)
    throw(ErrorException("get_position not implemented for trajectory of type $(typeof(agent))"))
end

function propagate_along_trajectory!(agent, model)
    throw(ErrorException("propagate_along_trajectory! not implemented for $(typeof(agent))"))
end

function validate_candidate(candidate_trajectory, agent, model)::Bool
    println("Validating candidate over time $(scaled_time(model)) to $(candidate_trajectory.t_bak)")

    for t in scaled_time(model):model.dt:candidate_trajectory.t_bak
        pos = get_position(candidate_trajectory, t)
        for neighbor in comms_radius(agent, model)
            if squared_dist(pos, get_position(neighbor.committed_trajectory, t)) < model.delta^2
                return false
            end
        end
    end

    return true
end


NEIGHBOR_TRAJECTORIES = []
DELTA = 1.0
function isValid(time::Float64, x::Float64, y::Float64)::Cint
    global NEIGHBOR_TRAJECTORIES

    # Check if the position (x, y) is valid at time t
    for agent_trajectory in NEIGHBOR_TRAJECTORIES
        pos = get_position(agent_trajectory, time)
        dist_sq = (x - pos[1])^2 + (y - pos[2])^2

        if dist_sq < DELTA^2
            return Cint(0) # Invalid position
        end
    end

    return Cint(1)
end

function make_valid_function(neighbor_trajectories::VCT, model) where {CT<:AbstractCompositeTrajectory,VCT<:AbstractVector{CT}}
    global NEIGHBOR_TRAJECTORIES = neighbor_trajectories
    global DELTA = model.delta
    return @cfunction(isValid, Cint, (Cdouble, Cdouble, Cdouble))
end


end # module Gatekeeper