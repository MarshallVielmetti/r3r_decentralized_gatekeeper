using Agents, DataFrames, Statistics, Random, LinearAlgebra, StaticArrays
using CairoMakie, Plots, Colors

using DynamicRRT, OccupancyGrids
using r3r

N_AGENTS = 4
center = (5.0, 5.0)
radius = 2.5

starting_positions = SVector{3, Float64}[]
goal_positions = SVector{3, Float64}[]
for i in 1:N_AGENTS
    angle = 2π * (i - 1) / N_AGENTS

    x_start = center[1] + radius * cos(angle)
    y_start = center[2] + radius * sin(angle)

    x_goal = center[1] + radius * cos(angle + π)
    y_goal = center[2] + radius * sin(angle + π)

    push!(starting_positions, SVector{3, Float64}(x_start, y_start, angle + π))
    push!(goal_positions, SVector{3, Float64}(x_goal, y_goal, angle + π))
end

random_seed = 2234123
rcomm = 4.0

rrt_iterations = 50
rrt_timeout = 1.0
rrt_early_exit = false

turning_radius = 0.25
delta = 0.01
model = r3r.init_dubins_agent_2d_problem(;starting_positions=starting_positions, goal_positions=goal_positions,n_agents=N_AGENTS, seed=random_seed, delta=delta, Rcomm=rcomm, turning_radius=turning_radius, dim=10.0, rrt_iterations=rrt_iterations, rrt_timeout=rrt_timeout, rrt_early_exit=rrt_early_exit``) # , occupancy_grid=ogm, sample_grid=inflated_ogm)

# Take an initial step
till_done = (model_in, t) -> nagents(model_in) == 0  || t > 1.0
agent_df, model_df = run!(model, till_done)

function plot_system_state(model, title_text="Current System Configuration", legend=false, resolution=(1000, 1000), title_fontsize=30, ax_pre=nothing)
    ax, fig = nothing, nothing
    if !isnothing(ax_pre)
        ax = ax_pre
    else
        fig = Figure(resolution=resolution)
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

    # Generate distinct colors for each agent
    n_agents = N_AGENTS
    colors = [RGB(HSV(360 * (i - 1) / n_agents, 0.8, 0.9)) for i in 1:n_agents]
    
    # Circle parameters for visualization
    circle_points = 50
    θ = range(0, 2π, length=circle_points)
    
    # Plot each agent
    for i in 1:n_agents
        agent = nothing
        try
            agent = model[i]
        catch
            continue
        end
        agent_color = colors[i]
        
        # Current position
        pos = Point2f(agent.pos[1], agent.pos[2])
        scatter!(ax, [pos], color=agent_color, markersize=12, label="Agent $i")
        
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
        comm_circle_x = agent.pos[1] .+ model.Rcomm .* cos.(θ)
        comm_circle_y = agent.pos[2] .+ model.Rcomm .* sin.(θ)
        lines!(ax, comm_circle_x, comm_circle_y, 
            color=agent_color, 
            alpha=0.3, 
            linewidth=1, 
            linestyle=:dot
        )
        
        # Planning radius circle (R-boundedness demonstration)
        position_at_time_committed = r3r.Gatekeeper.get_position(agent.committed_trajectory, agent.committed_trajectory.t_committed)
        # plan_circle_x = agent.pos[1] .+ model.Rplan .* cos.(θ)
        # plan_circle_y = agent.pos[2] .+ model.Rplan .* sin.(θ)
        plan_circle_x = position_at_time_committed[1] .+ model.Rplan .* cos.(θ)
        plan_circle_y = position_at_time_committed[2] .+ model.Rplan .* sin.(θ)
        lines!(ax, plan_circle_x, plan_circle_y, 
            color=agent_color, 
            alpha=0.5, 
            linewidth=2, 
            linestyle=:dashdot
        )
        
        # Plot committed trajectory if it exists
        if !isnothing(agent.committed_trajectory)
            trajectory_points = []
            t_sample = 0.0:0.1:10.0  # Sample trajectory for 2 time units

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


fig = Figure(resolution=(4000, 2000))

for i in 1:8
    title_text ="System State after $(i*50) time units (t=$(round(abmtime(model), digits=2)))" 
    title_fontsize = 35

    test = fig[floor(Int, (i-1) / 4) + 1, ((i -1) % 4) + 1] 

    ax = Axis(test,
        xlabel="X", 
        ylabel="Y", 
        title=title_text,
        titlesize=title_fontsize,
        aspect=DataAspect()
    )
    plot_system_state(model, "System State after $(i*50) time units (t=$(round(abmtime(model), digits=2)))", false, (1000, 1000), 16, ax)

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 125.0
    agent_df, model_df = run!(model, till_done)
end

save("pretty_system_summary.png", fig)