

function any_agents_collide(model)
    for (i, agent1) in enumerate(allagents(model))
        for (j, agent2) in enumerate(allagents(model))
            if i < j  # Avoid double checking pairs and self-collision
                if r3r.squared_dist(agent1.pos, agent2.pos) < (model.delta)^2
                    println("Collision Between Agent $(agent1.id) and Agent $(agent2.id)")
                    return true
                end
            end
        end
    end
    return false
end


function run_nmpc_trial(n_agents, Rcomm, random_seed, params)
    radius = get(params, :circle_radius, 2.5)
    center = (5.0, 5.0)

    starting_positions = SVector{3, Float64}[]
    goal_positions = SVector{3, Float64}[]
    for i in 1:n_agents
        angle = 2π * (i - 1) / n_agents

        x_start = center[1] + radius * cos(angle)
        y_start = center[2] + radius * sin(angle)

        x_goal = center[1] + radius * cos(angle + π)
        y_goal = center[2] + radius * sin(angle + π)

        push!(starting_positions, SVector{3, Float64}(x_start, y_start, angle + π))
        push!(goal_positions, SVector{3, Float64}(x_goal, y_goal, angle + π))
    end

    delta = get(params, :delta, 0.5)
    Rgoal = get(params, :Rgoal, 0.5)
    turning_radius = get(params, :turning_radius, 0.25)

    model = ipopt_baseline_model(;starting_positions=starting_positions, goal_positions=goal_positions,n_agents=n_agents, seed=random_seed, delta=delta, Rcomm=Rcomm, Rgoal=Rgoal,turning_radius=turning_radius, dim=10.0)

    # Step through model, checking if collision constraints are violated at any time.
    # If so, return false. Otherwise, return true

    max_steps = get(params, :max_steps, 200)
    for t in 1:max_steps
        if any_agents_collide(model)
            println("Collision Detected at time $t")
            return false
        end

        step!(model)

        if isempty(allagents(model))
            # All agents have reached their goals
            break
        end
    end

    return true
end
