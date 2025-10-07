using Agents, DataFrames, Statistics, Random, LinearAlgebra, StaticArrays
using CairoMakie, Colors, ColorSchemes

using DynamicRRT, OccupancyGrids
using r3r

# N_AGENTS = 128
N_AGENTS = 32
# center = (5.0, 5.0)
# radius = 2.5

# starting_positions = SVector{3, Float64}[]
# goal_positions = SVector{3, Float64}[]
# for i in 1:N_AGENTS
#     angle = 2π * (i - 1) / N_AGENTS

#     x_start = center[1] + radius * cos(angle)
#     y_start = center[2] + radius * sin(angle)

#     x_goal = center[1] + radius * cos(angle + π)
#     y_goal = center[2] + radius * sin(angle + π)

#     push!(starting_positions, SVector{3, Float64}(x_start, y_start, angle + π))
#     push!(goal_positions, SVector{3, Float64}(x_goal, y_goal, angle + π))
# end

ogm = load_grid(SimpleIndoor1Large; inflation=0.00, compute_sdf=true)
inflated_ogm = load_grid(SimpleIndoor1Large; inflation=3.0, compute_sdf=true)

# random_seed = 2234123
random_seed = 3234123
rcomm = 32.0

rrt_iterations = 1000
rrt_timeout = 5.0
rrt_early_exit = true

turning_radius = 0.5
delta = 0.5

# Simulation time step
dt = 0.2
min_replan_cooldown = 50  # Minimum steps between replans for each agent

model = r3r.init_dubins_agent_2d_problem(;n_agents=N_AGENTS, dt=dt, seed=random_seed, delta=delta, Rcomm=rcomm, turning_radius=turning_radius, rrt_iterations=rrt_iterations, rrt_timeout=rrt_timeout, rrt_early_exit=rrt_early_exit, occupancy_grid=ogm, sample_grid=inflated_ogm, min_replan_cooldown=20) # , occupancy_grid=ogm, sample_grid=inflated_ogm)

# Take an initial step
# till_done = (model_in, t) -> nagents(model_in) == 0  || t > 1.0
# agent_df, model_df = run!(model, till_done)
for _ in 1:50
    step!(model)
end

function plot_system_state(model, title_text="Current System Configuration", legend=false, resolution=(1000, 1000), title_fontsize=30, ax_pre=nothing)
    ax, fig = nothing, nothing
    if !isnothing(ax_pre)
        ax = ax_pre
    else
        fig = Figure(;resolution=resolution)
        ax = Axis(fig[1, 1], 
            xlabel="X", 
            ylabel="Y", 
            title=title_text,
            titlesize=title_fontsize,
            aspect=DataAspect()
        )
    end
    
    # Set axis limits
    xlims!(ax, 0 + 2., model.dims[1] - 2.)
    ylims!(ax, 0 + 2., model.dims[2] - 2.)

    if !isnothing(model.occupancy_grid) && true
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

    # Generate distinct colors for each agent using the tab10 palette (colorblind friendly)
    n_agents = N_AGENTS
    base_palette = collect(ColorSchemes.tab10.colors)
    if n_agents > length(base_palette)
        # Fallback: cycle if more agents than palette size
        colors = [base_palette[(i - 1) % length(base_palette) + 1] for i in 1:n_agents]
    else
        colors = base_palette[1:n_agents]
    end
    
    # Circle parameters for visualization
    circle_points = 50
    θ = range(0, 2π, length=circle_points)
    
    # Storage for computing closest pair
    agent_positions = Point2f[]
    agent_ids = Int[]

    # Plot each agent
    for i in 1:n_agents
        agent = nothing
        try
            agent = model[i]
        catch
            continue
        end

        if isnothing(agent.committed_trajectory)
            continue
        end

        agent_color = colors[i]
        
        # Current position
        pos = Point2f(agent.pos[1], agent.pos[2])
        scatter!(ax, [pos], color=agent_color, markersize=12, label="Agent $i")
        push!(agent_positions, pos)
        push!(agent_ids, i)
        
        # Goal position  
        goal_pos = Point2f(agent.goal[1], agent.goal[2])
        scatter!(ax, [goal_pos], 
            color=agent_color, 
            marker=:star5, 
            markersize=10, 
            strokecolor=:black, 
            strokewidth=1
        )
        
        # Direction vector (orientation)
        theta = agent.theta

        # Draw arrow showing orientation
        arrow_length = 0.3
        end_x = agent.pos[1] + arrow_length * cos(theta)
        end_y = agent.pos[2] + arrow_length * sin(theta)
        
        arrows!(ax, 
            [agent.pos[1]], [agent.pos[2]], 
            [arrow_length * cos(theta)], [arrow_length * sin(theta)],
            color=agent_color, 
            linewidth=2,
            arrowsize=12
        )
        
        # Communication radius circle
        # comm_circle_x = agent.pos[1] .+ model.Rcomm .* cos.(θ)
        # comm_circle_y = agent.pos[2] .+ model.Rcomm .* sin.(θ)
        # lines!(ax, comm_circle_x, comm_circle_y, 
        #     color=agent_color, 
        #     alpha=0.3, 
        #     linewidth=1, 
        #     linestyle=:dot
        # )
        
        # Planning radius circle (R-boundedness demonstration)
        position_at_time_committed = r3r.Gatekeeper.get_position(agent.committed_trajectory, agent.committed_trajectory.t_committed)
        # plan_circle_x = agent.pos[1] .+ model.Rplan .* cos.(θ)
        # plan_circle_y = agent.pos[2] .+ model.Rplan .* sin.(θ)
        plan_circle_x = position_at_time_committed[1] .+ model.Rplan .* cos.(θ)
        plan_circle_y = position_at_time_committed[2] .+ model.Rplan .* sin.(θ)

        # Draw planning radius as a semi-transparent filled disk (visual emphasis with low clutter)
        plan_points = Point2f.(plan_circle_x, plan_circle_y)
        poly!(ax, plan_points; color=(agent_color, 0.08), strokecolor=(agent_color, 0.4), strokewidth=1)
        
        # Plot committed trajectory if it exists
        if !isnothing(agent.committed_trajectory)
            trajectory_points = []
            t_sample = 0.0:0.1:20.0  # Sample trajectory for 2 time units

            for t in t_sample
                try
                    pos_t = r3r.Gatekeeper.get_position(agent.committed_trajectory, r3r.scaled_time(model) + t)
                    if !any(isnan.(pos_t))
                        push!(trajectory_points, Point2f(pos_t[1], pos_t[2]))
                    end
                catch
                    # Handle case where trajectory sampling fails
                    break
                end
            end
            
            if length(trajectory_points) > 1
                lines!(ax, trajectory_points, 
                    color=agent_color, 
                    linewidth=3, 
                    alpha=0.8
                )
            end
        end
    end

    # After plotting agents, draw line between closest pair with distance annotation
    if length(agent_positions) >= 2
        min_d = Inf
        idx_a, idx_b = 0, 0
        for a in 1:length(agent_positions)-1
            for b in a+1:length(agent_positions)
                p1 = agent_positions[a]; p2 = agent_positions[b]
                d = hypot(p1[1]-p2[1], p1[2]-p2[2])
                if d < min_d
                    min_d = d; idx_a = a; idx_b = b
                end
            end
        end
        if isfinite(min_d)
            p1 = agent_positions[idx_a]; p2 = agent_positions[idx_b]
            points = [Point2f(p1[1], p1[2]), Point2f(p2[1], p2[2])]
            lines!(ax, points; color=:black, linewidth=1, alpha=0.6)

            midx = (p1[1] + p2[1]) / 2
            midy = (p1[2] + p2[2]) / 2
            text!(ax, "$(round(min_d, digits=2))"; position=(midx, midy), 
                align=(:center, :bottom), fontsize=14, color=:black)
        end
    end
    
    if legend
        # Add custom legend elements
        legend_elements = [
            MarkerElement(color=:gray, marker=:circle, markersize=12),
            MarkerElement(color=:gray, marker=:star5, markersize=10),
            LineElement(color=:gray, linestyle=:solid, linewidth=3),
            MarkerElement(color=:gray, marker=:diamond, markersize=8),
            LineElement(color=:gray, linestyle=:dashdot, linewidth=2),
            LineElement(color=:gray, linestyle=:dot, linewidth=1),
            LineElement(color=:gray, linestyle=:dash, linewidth=1, alpha=0.4)
        ]
        
        legend_labels = [
            "Agent Position",
            "Goal Position", 
            "Committed Trajectory",
            "Backup Set",
            "Planning Radius (R-bounded)",
            "Communication Radius",
            "Communication Links"
        ]
    
        Legend(fig[1, 2], legend_elements, legend_labels, "Legend", tellheight=false)
    end
    return fig
end


"""
Generate a publication-style multi-panel snapshot figure.
"""
fig = Figure(resolution=(1500, 1000), padding=(5,5,5,5))

# Optional: adjust inter-panel gaps (nearly flush but a hairline for separation)
colgap!(fig.layout, 8)
rowgap!(fig.layout, 8)

for i in 1:6
        # Map panel index to 2x3 grid
        gl = fig[floor(Int, (i-1) / 3) + 1, ((i -1) % 3) + 1]
        ax = Axis(gl; aspect=DataAspect())
        # Hide all ticks & grids – retain clean white canvas
        hidedecorations!(ax)  # hides ticks, ticklabels, grid
        # hidespines!(ax)

        # Plot system state on this axis (title args suppressed because axis pre-supplied)
        plot_system_state(model, "", false, (1000, 1000), 16, ax)

        # Advance simulation between snapshots
        till_done = (model_in, t) -> nagents(model_in) == 0  || t > 300.0
        agent_df, model_df = run!(model, till_done)
end

save("pretty_system_summary.png", fig)
# save("pretty_system_summary.pdf", fig)