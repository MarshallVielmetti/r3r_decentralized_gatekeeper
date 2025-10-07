using JuMP, Ipopt
using StaticArrays

@agent struct MpcAgent(ContinuousAgent{2,Float64})
    theta::Float64
    goal::SVector{2,Float64}
    omega_max::Float64
    trajectory::Union{Nothing,Matrix{Float64}} # Predicted trajectory from MPC
    u_trajectory::Union{Nothing,Matrix{Float64}} # Predicted control inputs from MPC
    in_network::Bool
end

function mpc_agent_step!(agent::MpcAgent, model)
    mpc_problem = nl_mpc(agent, model)
    optimize!(mpc_problem)
    status = termination_status(mpc_problem)

    if status == MOI.OPTIMAL || status == MOI.LOCALLY_SOLVED
        x_opt = value.(mpc_problem[:x])
        u_opt = value.(mpc_problem[:u])

        agent.trajectory = x_opt
        agent.u_trajectory = u_opt

        move_agent!(agent, x_opt[SOneTo(2), 2], model)
        # agent.pos = SVector{2,Float64}(x_opt[1, 2], x_opt[2, 2])
        agent.theta = x_opt[3, 2]
    else
        @warn "MPC optimization failed for agent $(agent.id) with status $status"
        relaxed_problem = nl_mpc_relaxed(agent, model)
        optimize!(relaxed_problem)

        status = termination_status(relaxed_problem)
        if status == MOI.OPTIMAL || status == MOI.LOCALLY_SOLVED
            x_opt = value.(relaxed_problem[:x])
            u_opt = value.(relaxed_problem[:u])

            agent.trajectory = x_opt
            agent.u_trajectory = u_opt

            move_agent!(agent, x_opt[SOneTo(2), 2], model)
            agent.theta = x_opt[3, 2]
        else
            @warn "Relaxed MPC optimization also failed for agent $(agent.id) with status $status. Agent will not move this step."
        end
    end

    # Check if the agent is at its goal
    if r3r.squared_dist(agent.pos, agent.goal) < model.Rgoal^2
        println("Current agent IDs: ", [agent.id for agent in allagents(model)])
        remove_agent!(agent, model)
    end
    
end

function mpc_model_step!(model)
end

function nl_mpc(agent, model)
    n_states = 3 # x, y, theta
    n_controls = 1 # omega

    N = 40 # Prediction Horizon
    dt = model.dt # Timestep
    # dt = model.dt

    mpc = Model(Ipopt.Optimizer)
    set_silent(mpc)

    @variable(mpc, x[1:n_states, 1:N+1]) # State variables
    @variable(mpc, u[1:n_controls, 1:N]) # Control variables

    # Warm start with previous solution if available
    if !isnothing(agent.trajectory) && !isnothing(agent.u_trajectory)
        # Shift previous solution and extrapolate
        for i in 1:n_states
            for t in 1:N
                if t < N
                    # Shift previous trajectory
                    set_start_value(x[i, t+1], agent.trajectory[i, t+1])
                else
                    # Extrapolate last state
                    set_start_value(x[i, t+1], agent.trajectory[i, end])
                end
            end
        end
        
        for t in 1:N-1
            # Shift previous control inputs
            set_start_value(u[1, t], agent.u_trajectory[1, t+1])
        end
        # Extrapolate last control input
        set_start_value(u[1, N], agent.u_trajectory[1, end])
    end

    # Initial conditions
    x0 = [agent.pos[1]; agent.pos[2]; agent.theta]
    @constraint(mpc, x[:, 1] .== x0)
    
    # Input Constraints
    @constraint(mpc, -agent.omega_max .<= u[1, :] .<= agent.omega_max)

    # Inter-agent collision-avoidance constraints
    for neighbor in r3r.comms_radius(agent, model)
        if isnothing(neighbor.trajectory)
            continue
        end

        for t in 1:N+1
            neighbor_pos = neighbor.trajectory[1:2, t]
            @constraint(mpc, (x[1, t] - neighbor_pos[1])^2 + (x[2, t] - neighbor_pos[2])^2 >= (model.delta + 0.01)^2)
        end
    end

    # Dynamics
    v = 1.0  # constant forward velocity for Dubins car
    for t in 1:N
        @constraint(mpc, x[:, t+1] .== x[:, t] + dt * [v * cos(x[3, t]); v * sin(x[3, t]); u[1, t]])
    end

    Q = 1.0
    Qf = 100.0
    R = 0.1

    # Objective
    @objective(mpc, Min,
        R * sum(u[1, t]^2 for t in 1:N) +
        Q * sum((x[1, t] - agent.goal[1])^2 + (x[2, t] - agent.goal[2])^2 for t in 1:N) +
        Qf * ((x[1, N+1] - agent.goal[1])^2 + (x[2, N+1] - agent.goal[2])^2)
    )
    # @objective(mpc, Min, sum(u[1, t]^2 for t in 1:N) + sum((x[1, t] - agent.goal[1])^2 + (x[2, t] - agent.goal[2])^2 for t in 1:N+1))

    return mpc
end

function nl_mpc_relaxed(agent, model; tol=1e-2)
    n_states = 3 # x, y, theta
    n_controls = 1 # omega

    N = 40 # Prediction Horizon
    dt = model.dt # Timestep
    # dt = model.dt

    mpc = Model(Ipopt.Optimizer)
    set_silent(mpc)

    # Relaxed solver settings for better convergence
    set_optimizer_attribute(mpc, "tol", tol)
    set_optimizer_attribute(mpc, "max_iter", 500)  # Increase iteration limit
    set_optimizer_attribute(mpc, "acceptable_tol", 1e-1)  # More lenient tolerance
    set_optimizer_attribute(mpc, "acceptable_iter", 15)  # Accept solution after 15 iterations

    @variable(mpc, x[1:n_states, 1:N+1]) # State variables
    @variable(mpc, u[1:n_controls, 1:N]) # Control variables

    # Warm start with previous solution if available
    if !isnothing(agent.trajectory) && !isnothing(agent.u_trajectory)
        # Shift previous solution and extrapolate
        for i in 1:n_states
            for t in 1:N
                if t < N
                    # Shift previous trajectory
                    set_start_value(x[i, t+1], agent.trajectory[i, t+1])
                else
                    # Extrapolate last state
                    set_start_value(x[i, t+1], agent.trajectory[i, end])
                end
            end
        end
        
        for t in 1:N-1
            # Shift previous control inputs
            set_start_value(u[1, t], agent.u_trajectory[1, t+1])
        end
        # Extrapolate last control input
        set_start_value(u[1, N], agent.u_trajectory[1, end])
    end

    # Initial conditions
    x0 = [agent.pos[1]; agent.pos[2]; agent.theta]
    @constraint(mpc, x[:, 1] .== x0)
    
    # Input Constraints
    @constraint(mpc, -agent.omega_max .<= u[1, :] .<= agent.omega_max)

    # Dynamics
    v = 1.0  # constant forward velocity for Dubins car
    for t in 1:N
        @constraint(mpc, x[:, t+1] .== x[:, t] + dt * [v * cos(x[3, t]); v * sin(x[3, t]); u[1, t]])
    end

    Q = 1.0
    Qf = 100.0
    R = 0.1
    Qcoll = 100.0  # Reduced penalty weight for better convergence

    # Collect collision avoidance penalty terms using smoother penalty
    collision_penalty = 0.0
    for neighbor in r3r.comms_radius(agent, model)
        if isnothing(neighbor.trajectory)
            continue
        end

        for t in 1:N+1
            neighbor_pos = neighbor.trajectory[1:2, t]
            dist_squared = (x[1, t] - neighbor_pos[1])^2 + (x[2, t] - neighbor_pos[2])^2
            # Smoother penalty function using exponential
            collision_penalty += Qcoll * exp(-dist_squared / (model.delta^2))
        end
    end

    # Objective with collision avoidance penalty
    @objective(mpc, Min,
        R * sum(u[1, t]^2 for t in 1:N) +
        Q * sum((x[1, t] - agent.goal[1])^2 + (x[2, t] - agent.goal[2])^2 for t in 1:N) +
        Qf * ((x[1, N+1] - agent.goal[1])^2 + (x[2, N+1] - agent.goal[2])^2) +
        collision_penalty
    )

    return mpc
end

function ipopt_baseline_model(;
    n_agents::Int=10, ## Number of agents
    delta::Float64=0.25, ## Inter-agent collision radius
    Rcomm::Float64=0.3, ## Communication Radius
    Rgoal::Float64=0.01, ## Goal Radius 
    dt::Float64=0.05, ## Time Step
    seed::Int=1234, ## Random Seed
    dim::Float64=2.0, ## Dimension of the world,
    turning_radius::Float64=0.05, ## Minimum turning radius for Dubins dynamics,
    starting_positions::Union{Nothing,Vector{SVector{3,Float64}}}=nothing, ## Starting positions of agents
    goal_positions::Union{Nothing,Vector{SVector{3,Float64}}}=nothing, ## Goal positions of agents
)
    dims = (dim, dim)
    v0 = @SVector [0.0,0.0]

    omega_max = 1.0 / turning_radius

    properties = Dict(
        :Rcomm => Rcomm,
        :Rplan => (Rcomm - delta) / 3.0,
        :delta => delta,
        :Rgoal => Rgoal,
        :dt => dt,
        :dims => dims,
        :turning_radius => turning_radius,
    )

    rng = Random.MersenneTwister(seed)

    # Create random starting and goal positions if not provided
    if isnothing(starting_positions) || isnothing(goal_positions)
        starting_positions = SVector{3, Float64}[]
        goal_positions = SVector{3, Float64}[]

        for i in 1:n_agents
            found_valid_position = false
            while !found_valid_position
                x_start = rand(rng) * (dim - 2.0) + 1.0
                y_start = rand(rng) * (dim - 2.0) + 1.0
                theta_start = rand(rng) * 2π

                x_goal = rand(rng) * (dim - 2.0) + 1.00
                y_goal = rand(rng) * (dim - 2.0) + 0.00

                # Ensure starting position not too close to existing agents
                # disgusting anti-pattern loop but whatever
                too_close = false
                for pos in starting_positions
                    if r3r.squared_dist(SVector{2, Float64}([x_start, y_start]), pos[SOneTo(2)]) < (2*delta)^2
                        too_close = true
                        break
                    end
                end

                if !too_close
                    found_valid_position = true
                    push!(starting_positions, SVector{3, Float64}(x_start, y_start, theta_start))
                    push!(goal_positions, SVector{3, Float64}(x_goal, y_goal, 0.0))
                end
            end
        end
    end


    model = StandardABM(MpcAgent, ContinuousSpace(dims; periodic=false, spacing=0.1); (agent_step!)=mpc_agent_step!, (model_step!)=mpc_model_step!, rng, properties)
    for (starting_position, goal_position) in zip(starting_positions, goal_positions)
        agent = MpcAgent(model, starting_position[SOneTo(2)], v0, starting_position[3], goal_position[SOneTo(2)], omega_max, nothing, nothing, true)
        add_agent_own_pos!(agent, model)
    end

    return model
end