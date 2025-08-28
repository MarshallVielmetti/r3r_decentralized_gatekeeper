module PlotUtils

using CairoMakie, DataFrames, Dubins, Colors

function animate_df(agent_df, model; fade_trails=false, step_size=1)
    fig = Figure()
    ax = Axis(fig[1, 1], xlabel="X", ylabel="Y", title="Agent Trajectories", aspect=DataAspect())

    # Get unique time steps and agent IDs
    all_unique_times = unique(agent_df.time)
    sort!(all_unique_times)

    # Filter times to include only multiples of step_size, plus the final time
    if step_size > 1
        indices_to_keep = [i for i in 1:step_size:length(all_unique_times)]
        # Ensure the final time is always included
        if length(all_unique_times) ∉ indices_to_keep
            push!(indices_to_keep, length(all_unique_times))
        end
        unique_times = all_unique_times[indices_to_keep]
    else
        unique_times = all_unique_times
    end
    unique_agents = unique(agent_df.id)
    n_agents = length(unique_agents)

    @show unique_times
    @show unique_agents
    @show n_agents

    # Set up axis limits
    xlims!(ax, 0, model.dims[1])
    ylims!(ax, 0, model.dims[2])  # Increased to accommodate 4 agents

    # Plot occupancy grid as background if it exists
    if !isnothing(model.occupancy_grid)
        # Get the occupancy grid data
        grid_data = model.occupancy_grid.data
        grid_resolution = model.occupancy_grid.grid_resolution

        # Create inverted colormap (0 = white, 1 = black)
        inverted_data = 1.0 .- grid_data

        # Calculate proper coordinate ranges
        n_rows, n_cols = size(grid_data)
        x_range = range(0, n_cols * grid_resolution, length=n_cols)
        y_range = range(0, n_rows * grid_resolution, length=n_rows)

        # Plot as heatmap background with proper coordinates
        heatmap!(ax, x_range, y_range, inverted_data', colormap=:grays, alpha=0.8)
    end

    # Define colors for agents - generate distinct RGB colors
    colors = [RGB(HSV(360 * (i - 1) / n_agents, 0.8, 0.9)) for i in 1:n_agents]

    # Create observables for animation - dynamically for all agents
    agent_positions = [Observable(Point2f[]) for _ in 1:n_agents]
    current_agents = [Observable(Point2f(0, 0)) for _ in 1:n_agents]

    # For fade trails, we'll use separate observables for trail points with alpha
    if fade_trails
        trail_points = [Observable(Point2f[]) for _ in 1:n_agents]
        trail_colors = [Observable(RGBAf[]) for _ in 1:n_agents]
    end

    # Plot agent goals (static, no need for observables)
    for i in 1:n_agents
        agent_goal = agent_df.goal[agent_df.id.==unique_agents[i]][1]
        goal_point = Point2f(agent_goal[1], agent_goal[2])
        scatter!(ax, [goal_point], color=colors[i], markersize=5)
    end


    # Create circles around agents (radius = 1.0)
    circle_points = 50
    RADIUS = model.delta / 2.0

    θ = range(0, 2π, length=circle_points)
    unit_circle_x = RADIUS .* cos.(θ)
    unit_circle_y = RADIUS .* sin.(θ)

    Rcomm = model.Rcomm
    comm_circle_x = Rcomm .* cos.(θ)
    comm_circle_y = Rcomm .* sin.(θ)

    # Create observables for circle positions
    agent_circles = [Observable([Point2f(x, y) for (x, y) in zip(unit_circle_x, unit_circle_y)]) for _ in 1:n_agents]
    comm_circles = [Observable([Point2f(x, y) for (x, y) in zip(unit_circle_x, unit_circle_y)]) for _ in 1:n_agents]

    # Plot trajectories, current positions, and circles for all agents
    for i in 1:n_agents
        agent_color = colors[i]

        if fade_trails
            # For fade trails, use scatter with color observable for alpha control
            scatter!(ax, trail_points[i], color=trail_colors[i], markersize=3, label="Agent $i")
        else
            # Plot trajectory (normal mode)
            lines!(ax, agent_positions[i], color=agent_color, linewidth=2, label="Agent $i")
        end

        # Plot current position
        scatter!(ax, current_agents[i], color=agent_color, markersize=8)

        # Plot circle around agent
        lines!(ax, agent_circles[i], color=agent_color, linewidth=1, alpha=0.5, linestyle=:dash)

        # Plot circle around agent
        lines!(ax, comm_circles[i], color=agent_color, linewidth=1, alpha=0.5, linestyle=:dash)
    end

    # axislegend(ax, position=:rt, tellwidth=false, tellheight=false)

    # Alternative: Create legend in separate area
    Legend(fig[1, 2], ax, "Agents", tellheight=false)

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
                    # Get all agent data up to current time
                    agent_data = filter(row -> row.id == agent_id && row.time <= t && row.in_network, agent_df)

                    if nrow(agent_data) > 0
                        positions = [Point2f(pos[1], pos[2]) for pos in agent_data.pos]

                        if fade_trails
                            # Update trail with fading colors
                            n_points = length(positions)
                            if n_points > 0
                                agent_color = colors[i]

                                # Create alpha values (newer points more opaque)
                                alphas = [max(0.2, j / n_points) for j in 1:n_points]
                                rgba_colors = [RGBAf(agent_color.r, agent_color.g, agent_color.b, alpha) for alpha in alphas]

                                # Update observables
                                trail_points[i][] = positions
                                trail_colors[i][] = rgba_colors
                            end
                        else
                            # Normal trail update
                            agent_positions[i][] = positions
                        end
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
                    if fade_trails
                        trail_points[i][] = Point2f[]
                        trail_colors[i][] = RGBAf[]
                    else
                        agent_positions[i][] = Point2f[]
                    end
                    current_agents[i][] = Point2f(NaN, NaN)  # Hide current position
                    agent_circles[i][] = Point2f[]  # Hide circle
                    comm_circles[i][] = Point2f[]  # Hide comm circle
                end
            end
        end
    end
end

function plot_switch_times(agent_df, model)
    fig = Figure()
    ax1 = Axis(fig[1, 1], title="Agent Commit Times", xlabel="Time", ylabel="t_com")
    ax2 = Axis(fig[2, 1], title="Agent Switch Times", xlabel="Time", ylabel="t_switch")
    ax3 = Axis(fig[3, 1], title="Agent Backup Times", xlabel="Time", ylabel="t_bak")

    for agent_id in unique(agent_df.id)
        agent_data = filter(row -> row.id == agent_id, agent_df)
        lines!(ax1, agent_data.time, agent_data.t_com, label="Agent $agent_id")
        if hasproperty(agent_data, :t_switch)
            lines!(ax2, agent_data.time, agent_data.t_switch, label="Agent $agent_id")
        end
        if hasproperty(agent_data, :t_bak)
            lines!(ax3, agent_data.time, agent_data.t_bak, label="Agent $agent_id")
        end
    end
    axislegend(ax1, position=:rt)
    axislegend(ax2, position=:rt)
    axislegend(ax3, position=:rt)
    return fig
end

function plot_agent_timing(agent_df, model, agent_id)
    fig = Figure()
    ax1 = Axis(fig[1, 1], title="Agent $agent_id Action Times", xlabel="Time", ylabel="Action Time")

    agent_data = filter(row -> row.id == agent_id, agent_df)
    lines!(ax1, agent_data.time, agent_data.t_com, label="T_Comm")
    if hasproperty(agent_data, :t_switch)
        lines!(ax1, agent_data.time, agent_data.t_switch, label="T_Switch")
    end
    if hasproperty(agent_data, :t_bak)
        lines!(ax1, agent_data.time, agent_data.t_bak, label="T_Bak")
    end

    axislegend(ax1, position=:rt)
    return fig
end

function plot_agent_at_time(agent_df, model, agent_id, t)
    fig = Figure()
    ax1 = Axis(fig[1, 1], title="Agent $agent_id at Time $t", xlabel="X", ylabel="Y")
    xlims!(ax1, 0, 1)
    ylims!(ax1, 0, 1)

    agent_data = filter(row -> row.id == agent_id && row.time == t, agent_df)
    # Failure case
    if nrow(agent_data) == 0
        return fig
    end

    # Plot agent position
    pos = agent_data.pos[1]
    scatter!(ax1, Point2f(pos[1], pos[2]), color=:blue, markersize=10, label="Position")

    if hasproperty(agent_data, :backup_set)
        # Plot backup set if it exists
        backup = agent_data.backup_set[1]
        scatter!(ax1, Point2f(backup[1], backup[2]), color=:red, markersize=5, label="Backup")
    end

    # Plot the nominal trajectory if it exists
    if hasproperty(agent_data, :nominal)
        pts = []
        for (path, _) in agent_data.nominal[1]
            path_len = dubins_path_length(path)

            for t in 0:0.01:1.0
                _, path_point = dubins_path_sample(path, t * path_len)
                push!(pts, Point2f(path_point[1], path_point[2]))
            end

        end
        lines!(ax1, pts, color=:red, label="Nominal Trajectory")
    end

    return fig
end

function plot_timing_statistics(stats)
    fig = Figure()
    ax = Axis(fig[1, 1], title="Avg. Time/Iteration vs Agent Density",
        xlabel="Agent Density (agents/dim²)", ylabel="Avg. Time/Agent Iteration (seconds)")

    # Extract data and calculate density using the dim field from stats
    n_agents = [stat.n_agents for stat in stats]
    runtimes = [stat.runtime for stat in stats]
    # dims = [stat.dim for stat in stats]
    # agent_density = n_agents ./ (dims .^ 2)

    # Plot
    scatter!(ax, n_agents, runtimes, color=:blue, markersize=8, label="Runtime")
    # lines!(ax, agent_density, runtimes, color=:blue, alpha=0.7)

    # Add grid and legend
    # grid!(ax)
    axislegend(ax, position=:rt)

    return fig
end

end