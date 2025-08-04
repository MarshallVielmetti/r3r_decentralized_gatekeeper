module PlotUtils

using CairoMakie, DataFrames

function animate_df(agent_df, model)
    fig = Figure()
    ax = Axis(fig[1, 1], xlabel="X", ylabel="Y", title="Agent Trajectories")

    # Get unique time steps and agent IDs
    unique_times = unique(agent_df.time)
    unique_agents = unique(agent_df.id)
    n_agents = length(unique_agents)

    @show unique_times
    @show unique_agents
    @show n_agents

    # Set up axis limits
    xlims!(ax, 0, 100)
    ylims!(ax, 0, 100)  # Increased to accommodate 4 agents

    # Define colors for agents
    colors = [:orange, :blue, :red, :green, :purple, :brown, :pink, :gray, :olive, :cyan]

    # Create observables for animation - dynamically for all agents
    agent_positions = [Observable(Point2f[]) for _ in 1:n_agents]
    current_agents = [Observable(Point2f(0, 0)) for _ in 1:n_agents]

    # Plot agent goals (static, no need for observables)
    # for i in 1:n_agents
    #     agent_goal = model[i].goal
    #     goal_point = Point2f(agent_goal[1], agent_goal[2])
    #     scatter!(ax, [goal_point], color=colors[mod1(i, length(colors))], markersize=5, label="Goal $i")
    # end


    # Create circles around agents (radius = 1.0)
    circle_points = 50
    RADIUS = model.delta

    θ = range(0, 2π, length=circle_points)
    unit_circle_x = RADIUS .* cos.(θ)
    unit_circle_y = RADIUS .* sin.(θ)

    comm_circle_x = 16. .* cos.(θ)
    comm_circle_y = 16. .* sin.(θ)

    # Create observables for circle positions
    agent_circles = [Observable([Point2f(x, y) for (x, y) in zip(unit_circle_x, unit_circle_y)]) for _ in 1:n_agents]
    comm_circles = [Observable([Point2f(x, y) for (x, y) in zip(unit_circle_x, unit_circle_y)]) for _ in 1:n_agents]

    # Plot trajectories, current positions, and circles for all agents
    for i in 1:n_agents
        agent_color = colors[mod1(i, length(colors))]

        # Plot trajectory
        lines!(ax, agent_positions[i], color=agent_color, linewidth=2, label="Agent $i")

        # Plot current position
        scatter!(ax, current_agents[i], color=agent_color, markersize=8)

        # Plot circle around agent
        lines!(ax, agent_circles[i], color=agent_color, linewidth=1, alpha=0.5, linestyle=:dash)

        # Plot circle around agent
        lines!(ax, comm_circles[i], color=agent_color, linewidth=1, alpha=0.5, linestyle=:dash)
    end

    axislegend(ax, position=:lt)

    # Animate
    record(fig, "agent_animation.gif", unique_times; framerate=30) do t
        # Get data for current time
        current_data = filter(row -> row.time == t, agent_df)

        if nrow(current_data) >= 1
            # Update trajectories and positions for each agent
            for i in 1:n_agents
                agent_id = unique_agents[i]

                # Only show agent if it's in the network at current time
                agent_current = filter(row -> row.id == agent_id, current_data)
                is_in_network = nrow(agent_current) > 0 && agent_current.in_network[1]

                if is_in_network
                    # Update trajectory up to current time (only for in-network agents)
                    agent_data = filter(row -> row.id == agent_id && row.time <= t && row.in_network, agent_df)
                    if nrow(agent_data) > 0
                        agent_positions[i][] = [Point2f(pos[1], pos[2]) for pos in agent_data.pos]
                    end

                    # Update current position with bounds checking
                    if length(agent_current.pos[1]) >= 2
                        agent_center = Point2f(agent_current.pos[1][1], agent_current.pos[1][2])
                        current_agents[i][] = agent_center

                        # Update circle position around agent
                        agent_circles[i][] = [Point2f(agent_center[1] + x, agent_center[2] + y) for (x, y) in zip(unit_circle_x, unit_circle_y)]

                        comm_circles[i][] = [Point2f(agent_center[1] + x, agent_center[2] + y) for (x, y) in zip(comm_circle_x, comm_circle_y)]
                    end
                else
                    # Hide agent by setting empty data
                    agent_positions[i][] = Point2f[]
                    current_agents[i][] = Point2f(NaN, NaN)  # Hide current position
                    agent_circles[i][] = Point2f[]  # Hide circle
                end
            end
        end
    end
end

end