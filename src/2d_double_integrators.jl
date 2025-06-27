module double_integrators_2d

using ..Gatekeeper2D

using Agents
using Agents.Pathfinding
using Colors

using CairoMakie

using StaticArrays, Random, LinearAlgebra

export init_model, plot_agent, plot_model_extras!

@agent struct DoubleIntegrator2D(ContinuousAgent{2,Float64})
    # has pos and vel built into the agent type
    in_network::Bool # Flag to indicate whether or not the agent has joined the network yet
    failed_last_replan::Bool # Flag to indicate if the agent failed to replan at last attempt
    my_neighbor_replanned::Bool # Flag to indicate if an agent's neighbor has replanned at the current time step
    goal::SVector{2,Float64} # Goal Position
    committed_trajectory::Union{Nothing,CompositeTrajectory{TN,TB}} where {TN,TB} # Composite trajectory for the agent or nothing
end

@kwdef mutable struct DoubleIntegratorParameters
    Rcomm::Float64 = 16.0 # Communication Radius 
    Rplan::Float64 = 5.0 # Planning Radius
    delta::Float64 = 1.0 # Inter-agent collision radius
    Rgoal::Float64 = 0.5 # Goal pose radius to prevent weird numerical instabilities
    v_max::Float64 = 1.0 # maximum velocity of an agent
    u_max::Float64 = 1.0 # maximum control input magnitude of an agent
    dt::Float64 = 0.01 # Simulation Time Step
    num_timesteps_before_replan::Int = 5 # Number of timesteps before agent reaches its switch time before replanning
end

"""
    model_step!(model)
Performs a single step of the model simulation.
"""
function model_step!(model)
    println("\n###################################")
    println("Model step at time $(scaled_time(model))")

    # Clear Replanning Flags
    for agent in allagents(model)
        agent.my_neighbor_replanned = false
    end

    return
end


"""
    agent_step!(agent, model)
Performs a single step of the agent simulation.
"""
function agent_step!(agent, model)
    println("Agent $(agent.id) step at time $(scaled_time(model))")

    if !agent.in_network
        # if the agent is not in the network, it should try to join
        if try_to_join_network!(agent, model)
            println("\tAgent $(agent.id) has joined the network successfully!")
            propagate_along_trajectory!(agent, model)
        end
        return
    end

    # nearby_agents = nearby_agents(model, agent, model.Rcomm)

    ## Check if agent is experiencing a collision (only consider in-network agents)
    # for neighbor in Iterators.filter(nearby_agents(model, agent, model.delta), n -> n.in_network)
    # An agent is inside the collision radius of this agent
    # nearby_set = nearby_agents(model, agent, model.delta)

    #     println("\tAgent $(agent.id) and agent $(neighbor.id) are too close!")
    #     println("\tThe distance between them is $(norm(agent.pos - neighbor.pos))")
    # end

    ## Check if the agent has reached its goal
    if norm(agent.pos - agent.goal) <= model.Rgoal
        # println("\tAgent $(agent.id) has reached its goal at $(agent.goal)")
        # Remove the agent from the model
        remove_agent!(agent, model) # TODO -- validate that calling this function here is safe
        return
    end

    ## CHECK REPLANNING
    # println("\tAgent $(agent.id) checking if it should replan...")
    if agent_should_replan(agent, model)
        # println("\tAgent $(agent.id) attempting to replan...")
        # Update the agent's composite trajectory
        candidate_trajectory = construct_candidate(agent, model)
        if candidate_trajectory !== nothing && validate_candidate(candidate_trajectory, agent, model)
            # If the candidate trajectory is valid, set it as the agent's committed trajectory
            # println("\tAgent $(agent.id) successfully constructed new committed trajectory!")
            update_committed_trajectory!(agent, model, candidate_trajectory)
            agent.failed_last_replan = false
        else
            # println("\tAgent $(agent.id) could not find a valid trajectory to commit.")
            agent.failed_last_replan = true
        end
    end

    ## Propagate Agent Along Committed Trajectory
    propagate_along_trajectory!(agent, model)
end


const v0 = @SVector [0.0, 0.0] # Initial velocity of agents

function init_model(;
    n_agents::Int=10, ## Number of agents
    delta::Float64=1.0, ## Inter-agent collision radius
    Rcomm::Float64=16.0, ## Planning Radius
    Rgoal::Float64=0.5, ## Goal Radius 
    dt::Float64=0.1, ## Time Step
    seed::Int=1234, ## Random Seed
    agent_starting_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing, ## Starting positions of agents
    agent_goal_positions::Union{Nothing,Vector{SVector{2,Float64}}}=nothing ## Goal positions of agents
)
    Rplan = (Rcomm - delta) / 3.0 # Formula for planning radius for weakly R-Bounded Planning
    dims = (100, 100) # Dimensions of the environment


    rng = MersenneTwister(seed)
    Random.seed!(seed)

    space = ContinuousSpace(dims; periodic=false)
    properties = DoubleIntegratorParameters(
        Rcomm=Rcomm,
        Rplan=Rplan,
        delta=delta,
        Rgoal=Rgoal,
        dt=dt
    )

    model = StandardABM(DoubleIntegrator2D, space; (agent_step!)=agent_step!, (model_step!)=model_step!, rng, properties)

    for i in 1:n_agents
        # while there are no obstacles:
        # pos = @SVector [rand(space.dims[1]), rand(space.dims[2])]
        # goal_pos = @SVector [rand(space.dims[1]), rand(space.dims[2])]

        if agent_starting_positions === nothing || agent_goal_positions === nothing
            pos = rand(Float64, 2) .* @SVector [dims[1], dims[2]]
            goal_pos = rand(Float64, 2) .* @SVector [dims[1], dims[2]]
        else
            pos = agent_starting_positions[i]
            goal_pos = agent_goal_positions[i]
        end

        # pos = rand(Float64, SVector{2,Float64}) .* @SVector [space.dims[1], space.dims[2]]
        # goal_pos = rand(Float64, SVector{2,Float64}) .* @SVector [space.dims[1], space.dims[2]]

        agent = (DoubleIntegrator2D)(model, pos, v0, false, false, false, goal_pos, nothing)

        add_agent_own_pos!(agent, model)
    end

    return model
end


# const ABMPlot = Agents.get_AMBPlot_type()
# function Agents.static_preplot!(ax::Axis, p::ABMPlot)

# end

function plot_agent(agent)
    """
    Agent plotting function compatible with abmplot from Agents.jl

    Returns marker properties for the agent based on its state:
    - If not in network: invisible (transparent)
    - If in network: colored marker based on trajectory status
    """

    if !agent.in_network
        # Return transparent marker for agents not in network
        # return :circle, 6, :transparent
        # color = RGBA(0, 0, 0, 0) # Transparent color
        color = :red
        return :circle
        # size = 6
        # return :●
    end

    # Choose color based on agent state
    if agent.committed_trajectory !== nothing
        # Agent has a committed trajectory - use blue
        color = :blue
        size = 10
    else
        # Agent is in network but no trajectory - use orange
        color = :orange
        size = 8
    end

    # return :x
    return :diamond
end

function agent_color(agent)
    """
    Returns the color for the agent based on its state.
    Used by abmplot to color agents in the plot.
    """
    if !agent.in_network
        return RGBA(0, 100, 0, 0.7) # Transparent color for agents not in network
    end

    return :black

    if agent.committed_trajectory !== nothing
        return :blue # Color for agents with committed trajectory
    else
        return :orange # Color for agents without committed trajectory
    end
end

function plot_model_extras!(ax, model)
    """
    Plot additional model elements like goals, backup sets, and planning radii
    This function is called by abmplot to add extra visualization elements
    """

    for agent in allagents(model)
        if !agent.in_network
            continue
        end
        agent_pos = Point2f(agent.pos[1], agent.pos[2])

        # Plot agent's goal
        goal_pos = Point2f(agent.goal[1], agent.goal[2])
        scatter!(ax, [goal_pos], color=:green, marker=:star5, markersize=12)

        # Plot the agent's communication radius Rcomm as a circle
        Rcomm = model.Rcomm
        θ = 0:0.1:2π
        comm_circle_x = agent_pos[1] .+ Rcomm .* cos.(θ)
        comm_circle_y = agent_pos[2] .+ Rcomm .* sin.(θ)
        comm_circle_points = [Point2f(x, y) for (x, y) in zip(comm_circle_x, comm_circle_y)]
        lines!(ax, comm_circle_points, color=(:blue, 0.3), linewidth=1, linestyle=:dash)

        # Plot trajectory-related elements if agent has committed trajectory
        if agent.committed_trajectory !== nothing
            committed_pos_ = agent.committed_trajectory.tracked_nominal_trajectory(agent.committed_trajectory.t_committed)
            committed_pos = Point2f(committed_pos_[1], committed_pos_[2])
            scatter!(ax, [committed_pos], color=:blue, marker=:circle, markersize=8,
                label="Agent $(agent.id) Committed Position")

            # Plot backup set as a circle
            backup_pos = Point2f(agent.committed_trajectory.backup_set[1:2]...)
            Rgoal = model.Rgoal

            θ = 0:0.1:2π
            backup_circle_x = backup_pos[1] .+ Rgoal .* cos.(θ)
            backup_circle_y = backup_pos[2] .+ Rgoal .* sin.(θ)
            backup_circle_points = [Point2f(x, y) for (x, y) in zip(backup_circle_x, backup_circle_y)]

            lines!(ax, backup_circle_points, color=(:red, 0.6), linewidth=2)

            # Plot planning radius around agent's position
            Rplan = model.Rplan
            plan_circle_x = committed_pos[1] .+ Rplan .* cos.(θ)
            plan_circle_y = committed_pos[2] .+ Rplan .* sin.(θ)
            plan_circle_points = [Point2f(x, y) for (x, y) in zip(plan_circle_x, plan_circle_y)]

            lines!(ax, plan_circle_points, color=(:orange, 0.3), linewidth=1, linestyle=:dash)

            # Plot the nominal trajectory as a line
            traj_points = [Point2f(agent.committed_trajectory.nominal_trajectory[1, i],
                agent.committed_trajectory.nominal_trajectory[2, i]) for i in 1:size(agent.committed_trajectory.nominal_trajectory)[2]]

            lines!(ax, traj_points, color=:blue, linewidth=2, linestyle=:dash, alpha=0.3,
                label="Agent $(agent.id) Trajectory")

            # Plot the tracked nominal trajectory, from the current time to the switch time
            tracked_nominal = agent.committed_trajectory.tracked_nominal_trajectory
            start_time = Float64(scaled_time(model))
            switch_time = agent.committed_trajectory.t_switch

            if tracked_nominal !== nothing
                # Assume tracked_nominal is an ODESolution with state [x, y, ...]
                times = start_time:0.1:switch_time
                # x_vals = tracked_nominal[1, :]
                # y_vals = tracked_nominal[2, :]
                # tracked_points = [Point2f(x, y) for (x, y) in zip(x_vals, y_vals)]
                tracked_points = [Point2f(tracked_nominal(t)[1], tracked_nominal(t)[2]) for t in times]
                @show tracked_points
                lines!(ax, tracked_points, color=:blue, linewidth=2, label="Tracked Nominal")
            end

            # Plot the backup trajectory, from the switch tiem to the backup time
            backup_trajectory = agent.committed_trajectory.backup_trajectory
            if backup_trajectory !== nothing
                # Assume backup_trajectory is an ODESolution with state [x, y, ...]
                backup_times = switch_time:0.1:agent.committed_trajectory.t_bak
                backup_points = [Point2f(backup_trajectory(t)[1], backup_trajectory(t)[2]) for t in backup_times]
                lines!(ax, backup_points, color=:red, linewidth=2, label="Backup Trajectory")
            end
        end
    end
end

end # module double_integrators_2d