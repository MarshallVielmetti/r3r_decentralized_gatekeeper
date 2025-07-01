module Gatekeeper2D

using ..STRRT_STAR
using ..R3RCommon

using Agents
using StaticArrays
using LinearAlgebra
using ControlSystemsBase
using OrdinaryDiffEq, DiffEqCallbacks

using JuMP, OSQP

export agent_should_replan, try_to_join_network!, construct_candidate, validate_candidate, update_committed_trajectory!, propagate_along_trajectory!
export get_control_input, get_position, reset_mpc_cache!, print_mpc_stats
export scaled_time


"""
    get_control_input(committed_trajectory, t)

Get the control input at time `t` from the committed trajectory.
"""
function get_control_input(trajectory::CompositeTrajectory, t::Float64)
    # Get the control input at time t from the nominal trajectory
    if t < trajectory.t_switch
        # return trajectory.nominal_trajectory(t)
        return trajectory.tracked_nominal_trajectory(t)[3:4]
    elseif t < trajectory.t_bak
        return trajectory.backup_trajectory(t)[3:4]
    else
        return @SVector [0.0, 0.0] # No control input after backup trajectory -- this is the backup controller
    end
end

function get_position(trajectory::CompositeTrajectory, t::Float64)
    # Get the position at time t from the nominal trajectory
    if t < trajectory.t_switch
        return trajectory.tracked_nominal_trajectory(t)[1:2]
    elseif t < trajectory.t_bak
        return trajectory.backup_trajectory(t)[1:2]
    else
        # return trajectory.backup_set[1:2] # Position at the backup set
        return trajectory.backup_trajectory[end][1:2]
    end
end


function construct_nominal_trajectory(agent, model)
    # Get the trajectories of all agents in the communication radius
    neighbor_agent_trajectories::Vector{CompositeTrajectory} = [neighbor.committed_trajectory for neighbor in nearby_agents(agent, model, model.Rcomm, search=:exact) if neighbor.in_network]

    stamped_start_pos = @SVector [agent.pos[1], agent.pos[2], scaled_time(model)]

    return plan_nominal(stamped_start_pos, agent.goal, make_valid_function(neighbor_agent_trajectories, model))
end

function closed_loop_tracking_nominal!(D, state, params, time)
    agent, nominal_path = params

    # Get the next waypoint on the nominal path
    idx = findfirst(x -> x[3] >= time, eachcol(nominal_path))
    if idx === nothing
        waypoint = agent.goal # If no waypoint found, use the goal
    else
        waypoint = nominal_path[:, idx]
    end

    # Apply simple PD control to track the waypoint
    pos_e = waypoint[1:2] - state[1:2] # position error
    vel_e = -state[3:4]

    error_term = vcat(pos_e, vel_e)

    A = @SMatrix [0.0 0.0 1.0 0.0;
        0.0 0.0 0.0 1.0;
        0.0 0.0 0.0 0.0;
        0.0 0.0 0.0 0.0]

    B = @SMatrix [0.0 0.0;
        0.0 0.0;
        1.0 0.0;
        0.0 1.0]

    model = ss(A, B, I, 0)
    L = lqr(model, Diagonal([50.0, 50.0, 0.0, 0.0]), Diagonal([1.0, 1.0]))

    u = L * error_term

    # normalize u
    if norm(u) > 1.0
        u = u / norm(u)
    end
    # u = u / norm(u) * min(norm(u), 1.0)

    if norm(state[3:4]) > 1.0
        state[3:4] = state[3:4] / norm(state[3:4])
    end

    # Dynamics
    D[1] = state[3] # dx/dt = vx
    D[2] = state[4] # dy/dt = vy
    D[3] = u[1] # dvx/dt = ax
    D[4] = u[2] # dvy/dt = ay
end

function closed_loop_tracking_backup!(D, state, params, time)
    agent, backup_set = params

    pos_e = backup_set[1:2] - state[1:2] # position error
    vel_e = -state[3:4] # Trying to drive the velocity to zero

    error_term = vcat(pos_e, vel_e)
    A = @SMatrix [0.0 0.0 1.0 0.0;
        0.0 0.0 0.0 1.0;
        0.0 0.0 0.0 0.0;
        0.0 0.0 0.0 0.0]

    B = @SMatrix [0.0 0.0;
        0.0 0.0;
        1.0 0.0;
        0.0 1.0]

    model = ss(A, B, I, 0)
    L = lqr(model, Diagonal([1.0, 1.0, 5.0, 5.0]), Diagonal([100.0, 100.0]))

    u = L * error_term

    # normalize u
    u = u / norm(u) * min(norm(u), 1.0)
    if norm(state[3:4]) > 1.0
        state[3:4] = state[3:4] / norm(state[3:4])
    end

    # Dynamics
    D[1] = state[3] # dx/dt = vx
    D[2] = state[4] # dy/dt = vy
    D[3] = u[1] # dvx/dt = ax
    D[4] = u[2]
end

function project_onto_polygon(v, A_dir, max_norm)
    """
    Project vector v onto the feasible region defined by polygon constraints:
    A_dir[i, :] * v <= max_norm for all i

    This solves a quadratic program to find the closest feasible point.
    """
    M = size(A_dir, 1)

    # Fast check if already feasible
    if norm(v) <= max_norm * 1.001  # Small tolerance for numerical errors
        return v
    end

    # Try simple scaling first (most common case)
    v_scaled = v * (max_norm / norm(v))

    # Check if simple scaling satisfies polygon constraints
    violations = A_dir * v_scaled .- max_norm
    if all(violations .<= 1e-10)  # All constraints satisfied
        return v_scaled
    end

    # Use QP projection for complex polygons
    try
        proj_model = Model(OSQP.Optimizer)
        set_silent(proj_model)

        # Minimal solver settings for speed
        set_optimizer_attribute(proj_model, "eps_abs", 1e-3)
        set_optimizer_attribute(proj_model, "eps_rel", 1e-3)
        set_optimizer_attribute(proj_model, "max_iter", 100)
        set_optimizer_attribute(proj_model, "polish", false)

        @variable(proj_model, v_proj[1:2])
        @objective(proj_model, Min, sum((v_proj[j] - v[j])^2 for j in 1:2))

        # Polygon constraints
        for i in 1:M
            @constraint(proj_model, A_dir[i, :] ⋅ v_proj <= max_norm)
        end

        optimize!(proj_model)

        if termination_status(proj_model) == MOI.OPTIMAL
            return value.(v_proj)
        else
            # Fallback: simple scaling
            return v_scaled
        end
    catch
        # Fallback: simple scaling to satisfy norm constraint
        return v_scaled
    end
end

# Global MPC cache for warm starting
mutable struct MPCCache
    osqp_model::Union{Nothing,Model}
    last_x_sol::Union{Nothing,Matrix{Float64}}
    last_u_sol::Union{Nothing,Matrix{Float64}}
    last_x_ref::Union{Nothing,SVector{4,Float64}}
    A_dir::Union{Nothing,Matrix{Float64}}
    nx::Int
    nu::Int
    N::Int
    M::Int
end

const MPC_CACHE = MPCCache(nothing, nothing, nothing, nothing, nothing, 4, 2, 20, 16)

function polygon_approximations(M::Int)::Matrix{Float64}
    angles = [2π * i / M for i in 0:(M-1)]
    return hcat([cos(a) for a in angles], [sin(a) for a in angles])
end

function setup_mpc_model(nx::Int, nu::Int, N::Int, M::Int,
    A::SMatrix{4,4,Float64}, B::SMatrix{4,2,Float64},
    Q::Diagonal{Float64}, R::Diagonal{Float64},
    A_dir::Matrix{Float64}, umax_norm::Float64,
    vmax_norm::Float64, RPlan::Float64, planning_origin::SVector{2,Float64})::Model

    osqp_model = Model(OSQP.Optimizer)
    set_silent(osqp_model)

    # Set OSQP solver options for optimal performance
    set_optimizer_attribute(osqp_model, "eps_abs", 1e-3)  # Slightly relaxed for speed
    set_optimizer_attribute(osqp_model, "eps_rel", 1e-3)
    set_optimizer_attribute(osqp_model, "max_iter", 500)  # Reduced iterations for real-time
    set_optimizer_attribute(osqp_model, "adaptive_rho", true)
    set_optimizer_attribute(osqp_model, "polish", false)  # Disable polishing for speed
    set_optimizer_attribute(osqp_model, "warm_start", true)  # Enable warm starting
    set_optimizer_attribute(osqp_model, "check_termination", 5)  # Check termination less frequently

    # Create model variables
    @variable(osqp_model, x[1:nx, 1:N+1])
    @variable(osqp_model, u[1:nu, 1:N])

    # Cost - use more efficient quadratic form with matrices
    @variable(osqp_model, x_ref[1:nx])

    # Vectorized cost formulation for better performance
    @expression(osqp_model, state_cost,
        sum(Q[i, i] * (x[i, k] - x_ref[i])^2 for i in 1:nx, k in 1:N))

    @expression(osqp_model, input_cost,
        sum(R[j, j] * u[j, k]^2 for j in 1:nu, k in 1:N))

    # Terminal cost
    @expression(osqp_model, terminal_cost,
        sum(Q[i, i] * (x[i, N+1] - x_ref[i])^2 for i in 1:nx))

    @objective(osqp_model, Min, state_cost + input_cost + terminal_cost)

    # Initial State constraint (will be updated each solve)
    @constraint(osqp_model, init_state[i=1:nx], x[i, 1] == 0.0)

    # Dynamics constraints - vectorized form
    @constraint(osqp_model, dynamics[k=1:N, i=1:nx],
        x[i, k+1] == sum(A[i, j] * x[j, k] for j in 1:nx) + sum(B[i, j] * u[j, k] for j in 1:nu))

    # Polygon constraints for input - more efficient formulation
    @constraint(osqp_model, input_poly[k=1:N, i=1:M],
        A_dir[i, 1] * u[1, k] + A_dir[i, 2] * u[2, k] <= umax_norm)

    # Polygon constraints for velocity - direct indexing
    @constraint(osqp_model, vel_poly[k=1:N+1, i=1:M],
        A_dir[i, 1] * x[3, k] + A_dir[i, 2] * x[4, k] <= vmax_norm)

    # Scaled Polygon Constraints for State - relative position constraints
    @constraint(osqp_model, state_poly[k=1:N+1, i=1:M],
        A_dir[i, 1] * (x[1, k] - x[1, 1]) + A_dir[i, 2] * (x[2, k] - x[2, 1]) <= RPlan)

    return osqp_model
end

function backup_mpc_controller!(D::AbstractVector{Float64}, state::AbstractVector{Float64},
    params::Tuple, time::Float64)::Nothing
    agent, backup_set = params

    # Constants
    DT::Float64 = 0.1
    N::Int = 20
    M::Int = 16
    RPlan::Float64 = 5.0
    nx::Int = 4
    nu::Int = 2

    # State-Space Model - discrete time
    A = @SMatrix Float64[1.0 0.0 DT 0.0;
        0.0 1.0 0.0 DT;
        0.0 0.0 1.0 0.0;
        0.0 0.0 0.0 1.0]

    B = @SMatrix Float64[0.0 0.0;
        0.0 0.0;
        DT 0.0;
        0.0 DT]

    # Cost matrices
    Q = Diagonal([100.0, 100.0, 10.0, 10.0])
    R = Diagonal([1.0, 1.0])

    # Constraint bounds
    umax_norm::Float64 = 1.0
    vmax_norm::Float64 = 1.0

    # Convert state to SVector for type safety
    x0 = @SVector Float64[state[1], state[2], state[3], state[4]]
    x_ref = @SVector Float64[backup_set[1], backup_set[2], backup_set[3], backup_set[4]]

    # Initialize cache if needed
    if MPC_CACHE.A_dir === nothing
        MPC_CACHE.A_dir = polygon_approximations(M)
        MPC_CACHE.osqp_model = setup_mpc_model(nx, nu, N, M, A, B, Q, R,
            MPC_CACHE.A_dir, umax_norm, vmax_norm, RPlan, agent.pos) # verify use of agent.pos
        MPC_CACHE.nx = nx
        MPC_CACHE.nu = nu
        MPC_CACHE.N = N
        MPC_CACHE.M = M
    end

    # Project initial velocity onto feasible region if needed
    v0_original = @SVector Float64[x0[3], x0[4]]
    v0_projected = project_onto_polygon(v0_original, MPC_CACHE.A_dir, vmax_norm)

    if norm(v0_projected - v0_original) > 1e-6
        x0 = @SVector Float64[x0[1], x0[2], v0_projected[1], v0_projected[2]]
    end

    osqp_model = MPC_CACHE.osqp_model

    # Update model with current state and reference
    # Update initial state constraint
    for i in 1:nx
        set_normalized_rhs(osqp_model[:init_state][i], x0[i])
    end

    # Update reference in objective
    for i in 1:nx
        fix(osqp_model[:x_ref][i], x_ref[i]; force=true)
    end

    # Warm start if we have previous solution
    warm_started = false
    if MPC_CACHE.last_x_sol !== nothing && MPC_CACHE.last_u_sol !== nothing &&
       norm(x0[1:2] - MPC_CACHE.last_x_ref[1:2]) < 2.0  # Only warm start if state hasn't changed too much

        warm_started = true
        # Shift previous solution for warm start
        x_warm = zeros(Float64, nx, N + 1)
        u_warm = zeros(Float64, nu, N)

        # Shift states (use previous solution shifted by one time step)
        x_warm[:, 1] = x0
        for k in 2:N
            if k < size(MPC_CACHE.last_x_sol, 2)
                x_warm[:, k] = MPC_CACHE.last_x_sol[:, k+1]
            else
                # Extrapolate using dynamics
                if k == 2
                    x_warm[:, k] = Vector(A * SVector{4,Float64}(x_warm[:, k-1]) + B * SVector{2,Float64}([0.0, 0.0]))
                else
                    x_warm[:, k] = Vector(A * SVector{4,Float64}(x_warm[:, k-1]) + B * SVector{2,Float64}(u_warm[:, k-2]))
                end
            end
        end
        x_warm[:, N+1] = x_ref

        # Shift controls
        for k in 1:N-1
            if k < size(MPC_CACHE.last_u_sol, 2)
                u_warm[:, k] = MPC_CACHE.last_u_sol[:, k+1]
            else
                u_warm[:, k] = [0.0, 0.0]  # Zero control for extrapolation
            end
        end
        u_warm[:, N] = [0.0, 0.0]

        # Set warm start values
        try
            for k in 1:N+1, i in 1:nx
                set_start_value(osqp_model[:x][i, k], x_warm[i, k])
            end
            for k in 1:N, j in 1:nu
                set_start_value(osqp_model[:u][j, k], u_warm[j, k])
            end
        catch e
            # If warm start fails, continue without it
            # println("\t\t\tWarm start failed: $e")
            warm_started = false
        end
    end

    # Optimize the model with timing
    solve_start_time = time_ns()
    optimize!(osqp_model)
    solve_time = (time_ns() - solve_start_time) / 1e9  # Convert to seconds

    # Update statistics
    MPC_STATS.total_solves += 1
    MPC_STATS.total_solve_time += solve_time
    if warm_started
        MPC_STATS.warm_start_successes += 1
    end

    # Check optimization result
    if termination_status(osqp_model) != MOI.OPTIMAL
        MPC_STATS.failed_solves += 1
        # println("\t\t\tOSQP optimization failed with status: $(termination_status(osqp_model))")
        # println("\t\t\tPrimal status: $(primal_status(osqp_model))")
        # println("\t\t\tDual status: $(dual_status(osqp_model))")

        # Try to recover the problem by clearing warm start and resolving
        if MPC_CACHE.last_x_sol !== nothing
            # println("\t\t\tClearing warm start and retrying...")
            # Clear warm start values
            for k in 1:N+1, i in 1:nx
                unset_start_value(osqp_model[:x][i, k])
            end
            for k in 1:N, j in 1:nu
                unset_start_value(osqp_model[:u][j, k])
            end

            retry_start_time = time_ns()
            optimize!(osqp_model)
            retry_time = (time_ns() - retry_start_time) / 1e9
            MPC_STATS.total_solve_time += retry_time

            if termination_status(osqp_model) != MOI.OPTIMAL
                # println("\t\t\tRetry failed. Using fallback control.")
            else
                # println("\t\t\tRetry successful!")
                MPC_STATS.successful_solves += 1
            end
        end

        # If still not optimal, use fallback
        if termination_status(osqp_model) != MOI.OPTIMAL
            # Fallback: use simple proportional control
            # println("\t\t\tUsing fallback proportional control")
            pos_error = backup_set[1:2] - state[1:2]
            vel_error = -state[3:4]

            u_fallback = 0.5 * pos_error + 0.1 * vel_error
            if norm(u_fallback) > umax_norm
                u_fallback = u_fallback / norm(u_fallback) * umax_norm
            end

            # Dynamics
            D[1] = state[3] # dx/dt = vx
            D[2] = state[4] # dy/dt = vy
            D[3] = u_fallback[1] # dvx/dt = ax
            D[4] = u_fallback[2] # dvy/dt = ay
            return
        end
    else
        MPC_STATS.successful_solves += 1
        # Track solver iterations if available
        try
            iter_count = get_optimizer_attribute(osqp_model, "iter_count")
            MPC_STATS.avg_iterations = (MPC_STATS.avg_iterations * (MPC_STATS.total_solves - 1) + iter_count) / MPC_STATS.total_solves
        catch
            # Iteration count not available
        end
    end

    # Extract optimal solution
    u_out = value.(osqp_model[:u][:, 1]) # Get the first control input 
    x_sol = value.(osqp_model[:x])
    u_sol = value.(osqp_model[:u])

    # Store solution for warm starting next iteration
    MPC_CACHE.last_x_sol = copy(x_sol)
    MPC_CACHE.last_u_sol = copy(u_sol)
    MPC_CACHE.last_x_ref = x_ref

    # Ensure control constraints are satisfied
    if norm(u_out) > umax_norm
        u_out = u_out / norm(u_out) * umax_norm
    end

    # Ensure velocity constraints are satisfied (project if needed)
    current_vel = @SVector Float64[state[3], state[4]]
    if norm(current_vel) > vmax_norm
        projected_vel = project_onto_polygon(current_vel, MPC_CACHE.A_dir, vmax_norm)
        state[3] = projected_vel[1]
        state[4] = projected_vel[2]
    end

    # Set dynamics
    D[1] = state[3] # dx/dt = vx
    D[2] = state[4] # dy/dt = vy
    D[3] = u_out[1] # dvx/dt = ax
    D[4] = u_out[2] # dvy/dt = ay

    return nothing
end

function forward_propagate_tracking_controller(agent, model, nominal_trajectory)
    # println("\t\tforward_propagate_tracking_controller()")
    # Forward propagate the tracking controller along the nominal trajectory 
    nominal_trajectory_end_time = nominal_trajectory[3, end]
    # println("\t\t\tNominal Trajectory End Time: $(nominal_trajectory_end_time)")

    # TODO paramterize the padding time (currently 5s)
    odeproblem = ODEProblem(closed_loop_tracking_nominal!, Vector([agent.pos; agent.vel]), (scaled_time(model), nominal_trajectory_end_time + 5.0), (agent, nominal_trajectory))

    # Forward prop until the agent leaves the planning radius
    function termination_condition(state, time, integrator)
        return norm(state[1:2] - agent.pos) - model.Rplan
    end

    # Stop the forward prop. if the agent collides with another agent or a static obstacle #TODO Static
    function interagent_collision_termination_condition(state, time, integrator)
        interagent_distances = [norm(state[1:2] - get_position(neighbor.committed_trajectory, time)) for neighbor in nearby_agents(agent, model, model.Rcomm, search=:exact) if neighbor.in_network]
        return min(Inf, interagent_distances...) - model.delta
    end

    function reached_goal_condition(state, time, integrator)
        # Check if the agent has reached the goal region
        return norm(state[1:2] - agent.goal) - (model.Rgoal / 2.)
    end

    termination_callback = ContinuousCallback(termination_condition, terminate!)
    collison_termination_callback = ContinuousCallback(interagent_collision_termination_condition, terminate!)
    reached_goal_callback = ContinuousCallback(reached_goal_condition, terminate!)

    callbacks = CallbackSet(termination_callback, collison_termination_callback, reached_goal_callback)

    odesol = solve(odeproblem, Tsit5(), dtmax=0.05, dt=0.05, callback=callbacks) # TODO make parameters passable

    return odesol
end

function choose_backup_set(agent, model, tracked_nominal_trajectory)
    nominal_end_time = tracked_nominal_trajectory.t[end]

    backup_time = nominal_end_time
    while norm(tracked_nominal_trajectory(backup_time)[1:2] - agent.pos) > model.Rplan && backup_time > tracked_nominal_trajectory.t[1]
        backup_time -= model.dt
    end

    # For now, we will just return the last point in the tracked nominal trajectory that is inside the planning radius
    # exit_point = tracked_nominal_trajectory(nominal_end_time - 1 * model.dt) # some cushion
    backup_point = tracked_nominal_trajectory(backup_time)
    backup_point[3] = 0.0 # Set the velocity to zero
    backup_point[4] = 0.0

    return backup_point
end

function construct_backup_trajectory(agent, model, tracked_nominal_trajectory, backup_set)
    # println("\t\tconstruct_backup_trajectory")
    # println("\t\tBackup Set: $(backup_set)")
    # Working backwards along the tracked nominal, find the latest point for which the agent can come to a complete stop inside the backup set
    # while requiring the agent remains inside the planning radius the whole time

    backup_trajectory = nothing
    t_switch = nothing

    end_time = tracked_nominal_trajectory.t[end]
    t_guess = end_time - 1.0 * model.dt# - 1.0

    function success_condition(state, time, integrator)
        # Check if the agent can come to a complete stop inside the backup set
        # pos = state[1:2]
        # vel = state[3:4]
        # return norm(pos - backup_set[1:2]) < model.Rgoal && norm(vel) < 0.01
        return 0.05 - norm(state - backup_set)
    end

    function failure_condition(state, time, integrator)
        # Check if the agent has left the planning radius
        # return norm(state[1:2] - agent.pos) < model.Rplan
        return (model.Rplan) - norm(state[1:2] - agent.pos)
    end

    current_nearby_agents = nearby_agents(agent, model, model.Rcomm, search=:exact)

    function interagent_collision_condition(state, time, integrator)
        interagent_distances = [norm(state[1:2] - get_position(neighbor.committed_trajectory, time)) for neighbor in current_nearby_agents if neighbor.in_network]
        return model.delta - min(Inf, interagent_distances...)
    end

    success_callback = ContinuousCallback(success_condition, terminate!)
    failure_callback = ContinuousCallback(failure_condition, terminate!)
    interagent_collision_callback = ContinuousCallback(interagent_collision_condition, terminate!)
    callback_set = CallbackSet(success_callback, failure_callback, interagent_collision_callback)

    successful = false
    while !successful && t_guess > tracked_nominal_trajectory.t[1]
        pos_at_t_guess = tracked_nominal_trajectory(t_guess)

        # println("\t\tt_guess = $(t_guess), starting_pose = $(pos_at_t_guess)")

        # odeproblem = ODEProblem(closed_loop_tracking_backup!, pos_at_t_guess, (t_guess, end_time + 5.0), (agent, backup_set))
        odeproblem = ODEProblem(backup_mpc_controller!, pos_at_t_guess, (t_guess, end_time + 5.0), (agent, backup_set))
        odesol = solve(odeproblem, Tsit5(), dt=0.05, dtmax=0.05, callback=callback_set) # TODO dt is hyperparam

        # println("\t\tTerminated at t=$(odesol.t[end]) with state $(odesol[end])")
        # println("\t\tNorm between final and goal: $(norm(odesol[end] - backup_set))")

        # Check if was successful
        if norm(odesol[end] - backup_set) ≤ 0.1
            # println("\t\tSUCCESS: Found a valid backup trajectory at t=$(t_guess) with state $(odesol[end])")
            # Success condition was met
            t_switch = t_guess
            backup_trajectory = odesol
            successful = true
        else
            t_guess = t_guess - 1. * model.dt
        end
    end

    return t_switch, backup_trajectory
end



function construct_candidate(agent, model)
    ## Steps to construct a candidate trajectory:
    # 1. If failed last replan (or no nominal) construct a nominal trajectory, terminating in the goal region, that does not collide with any neighboring agent's committed / static obstacles
    # 2. Forward propagate the tracking controller along the nominal trajectory to get the tracked nominal trajectory
    #    a. Only bother to forward propagate until the agent leaves the planning radius Rplan
    #    b. if the agent collides with another agent or a static obstacle, also stop forward propagation
    # 3. Choose a backup set
    #    a. For our purposes, the backup set can be the last point along the nominal trajectory inside the planning radius
    # 4. Working backwards, find the latest point for which the agent can come to a complete stop inside the backup set
    #    a. This needs to be dynamically feasible for the agent, i.e. the agent can decelerate to zero velocity at that point meeting constraints
    # 5. Validate the backup trajectory to be collision free

    ## In the following conditions, this function should return nothing:
    # 1. If the algorithm failed to consturct a valid nominal trajectory
    # 2. The algorithm failed to construct a valid backup trajectory

    # Step 1: Construct a nominal trajectory, safe relative to known information, that terminates in the goal region
    if (!agent.in_network || agent.failed_last_replan)
        nominal_trajectory = construct_nominal_trajectory(agent, model)
    else
        nominal_trajectory = agent.committed_trajectory.nominal_trajectory
    end

    if nominal_trajectory === nothing
        # println("\tAgent $(agent.id) could not construct a valid nominal trajectory!")
        return nothing
    end

    # println("\tAgent $(agent.id) constructed a valid nominal trajectory with $(size(nominal_trajectory, 2)) waypoints.")

    # Step 2: Forward propagate the tracking controller along the nominal trajectory
    tracked_nominal_trajectory = forward_propagate_tracking_controller(agent, model, nominal_trajectory)
    # The forward prop will stop when the agent leaves the planning radius or collides with another agent or a static obstacle

    # println("\tAgent $(agent.id) Successfully forward propagated the tracking controller along the nominal trajectory.")

    # Step 3: Choose a backup set
    backup_set = choose_backup_set(agent, model, tracked_nominal_trajectory)

    # println("\tAgent $(agent.id) chose a backup set: $(backup_set)")

    # Step 4: Working backwards along the tracked_nominal, find the latest point for which the agent can come to a complete stop inside the backup set
    t_switch, backup_trajectory = construct_backup_trajectory(agent, model, tracked_nominal_trajectory, backup_set)
    if backup_trajectory === nothing
        # println("\tAgent $(agent.id) could not construct a valid backup trajectory!")
        return nothing
    end
    t_bak = backup_trajectory.t[end]

    # Step 5: Validate the backup set is collision free after t_bak
    if !validate_backup_set(backup_set, t_bak, agent, model)
        # println("\tAgent $(agent.id) could not validate the backup trajectory!")
        return nothing
    end

    # Create a candidate trajectory
    candidate_trajectory = CompositeTrajectory(
        scaled_time(model), # The time at which the agent committed to this trajectory
        t_switch, # The time at which the agent switches from the nominal to the backup trajectory
        t_bak, # The time at which the agent enters the backup set
        nominal_trajectory,
        tracked_nominal_trajectory,
        backup_trajectory,
        backup_set
    )

    return candidate_trajectory
end

function validate_candidate(candidate_trajectory, agent, model)
    return true

    # Check if the candidate trajectory is valid
    # For now, we will just check if the trajectory is not empty
    return !isempty(candidate_trajectory.nominal_trajectory) && !isempty(candidate_trajectory.backup_trajectory)
end

# TODO I hate this
function validate_backup_set(backup_set, t_bak, agent, model)
    current_neighbors = nearby_agents(agent, model, model.Rcomm, search=:exact)

    for neighbor in current_neighbors
        if neighbor.in_network
            # Check if the neighbor is already in its backup set
            if (neighbor.committed_trajectory.t_bak > t_bak)
                # If so, then verify the backup sets are far enough apart
                # TODO -- Account for the fact that the backup set is a circle with some (small)radius not just a point
                if (norm(backup_set[1:2] - get_position(neighbor.committed_trajectory, t_bak)) < model.delta)
                    println("\t\tAgent $(agent.id) found a neighbor $(neighbor.id) that is too close at t_bak=$(t_bak).")
                    return false
                end
            else
                # Need to verify that the neighbor doesn't intersect with the backup set through t_bak
                for t in t_bak:model.dt:neighbor.committed_trajectory.t_bak
                    if norm(backup_set[1:2] - get_position(neighbor.committed_trajectory, t)) < model.delta
                        println("\t\tAgent $(agent.id) found a neighbor $(neighbor.id) that intersects with the backup set at t=$(t).")
                        return false
                    end
                end
            end
        end
    end

    return true
end

function update_committed_trajectory!(agent, model, candidate_trajectory)
    # Tell all neighbors of this agent that this agent has replanned, and thus they should replan
    # for neighbor in nearby_agents(agent, model, model.Rcomm, search=:exact)
    #     neighbor.my_neighbor_replanned = true
    # end
    # Set replanned flag so other agents know
    agent.just_replanned = true
    # Set the agent's committed trajectory to the candidate trajectory
    agent.committed_trajectory = candidate_trajectory
end

function propagate_along_trajectory!(agent, model)
    ## CHECK IF IN BACKUP SET -> SET VELOCITY TO ZERO TO AVOID WEIRD NUMERICAL ISSUES
    if scaled_time(model) >= agent.committed_trajectory.t_bak
        println("Agent Executing Backup")
        to_pose = agent.pos
        agent.vel = @SVector [0.0, 0.0] # Set the velocity to zero
        move_agent!(agent, to_pose, model) # Update the agent's position in the model
        return
    end

    to_pose = get_position(agent.committed_trajectory, scaled_time(model))
    move_agent!(agent, to_pose, model)
    return

    agent.pos = agent.pos + agent.vel * model.dt
    agent.vel = agent.vel + get_control_input(agent.committed_trajectory, scaled_time(model)) * model.dt
end

# Performance monitoring for MPC
mutable struct MPCStats
    total_solves::Int
    successful_solves::Int
    failed_solves::Int
    warm_start_successes::Int
    total_solve_time::Float64
    avg_iterations::Float64
end

const MPC_STATS = MPCStats(0, 0, 0, 0, 0.0, 0.0)

function reset_mpc_cache!()
    """Reset the MPC cache when simulation parameters change"""
    MPC_CACHE.osqp_model = nothing
    MPC_CACHE.last_x_sol = nothing
    MPC_CACHE.last_u_sol = nothing
    MPC_CACHE.last_x_ref = nothing
    MPC_CACHE.A_dir = nothing

    # Reset stats
    MPC_STATS.total_solves = 0
    MPC_STATS.successful_solves = 0
    MPC_STATS.failed_solves = 0
    MPC_STATS.warm_start_successes = 0
    MPC_STATS.total_solve_time = 0.0
    MPC_STATS.avg_iterations = 0.0
end

function print_mpc_stats()
    """Print MPC performance statistics"""
    if MPC_STATS.total_solves > 0
        success_rate = MPC_STATS.successful_solves / MPC_STATS.total_solves * 100
        warm_start_rate = MPC_STATS.warm_start_successes / MPC_STATS.total_solves * 100
        avg_time = MPC_STATS.total_solve_time / MPC_STATS.total_solves * 1000  # ms

        println("=== MPC Performance Statistics ===")
        println("Total solves: $(MPC_STATS.total_solves)")
        println("Success rate: $(round(success_rate, digits=1))%")
        println("Warm start rate: $(round(warm_start_rate, digits=1))%")
        println("Average solve time: $(round(avg_time, digits=2)) ms")
        println("Average iterations: $(round(MPC_STATS.avg_iterations, digits=1))")
    else
        println("No MPC statistics available")
    end
end

end # module Gatekeeper2D