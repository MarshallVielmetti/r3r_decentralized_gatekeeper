using Agents, DataFrames, Statistics, Random, LinearAlgebra, StaticArrays
using CairoMakie, Colors, LaTeXStrings
using Dubins
using r3r


function get_nominal_pose(trajectory, t)
    idx = findlast(x -> begin
            path, start_time = x
            start_time <= t
        end, trajectory.nominal_trajectory)
end

function pretty_plot_system(model, ax)
    # fig = Figure(resolution=(1000, 1000))
    # fig = Figure(resolution=(600, 300))

    # ax = Axis(fig[1, 1], 
    #     xlabel="X", 
    #     ylabel="Y", 
    #     # title="System State",
    #     aspect=DataAspect(),
    #     xgridvisible=false,
    #     ygridvisible=false
    # )

    # aspect!(ax, DataAspect())
    ax.aspect = DataAspect()
    hidedecorations!(ax)
    hidespines!(ax)
    tightlimits!(ax)
    
    # Set axis limits
    xlims!(ax, 0, model.dims[1])
    # ylims!(ax, 0, model.dims[2])
    ylims!(ax, 2.5, 6.5)

    # Generate distinct colors for each agent
    n_agents = length(allagents(model))
    colors = [RGB(HSV(360 * (i - 1) / n_agents, 0.8, 0.9)) for i in 1:n_agents]
    
    # Circle parameters for visualization
    circle_points = 50
    θ = range(0, 2π, length=circle_points)
    
    # Plot each agent
    for (i, agent) in enumerate(allagents(model))
        agent_color = colors[i]
        agent_idx = agent.id  # Use actual agent ID instead of enumeration index
        current_time = r3r.scaled_time(model)
        
        # 1. Plot executed trajectory (faded)
        if hasfield(typeof(agent), :executed_trajectory) && !isnothing(agent.executed_trajectory)
            executed_points = [Point2f(pt[1], pt[2]) for pt in agent.executed_trajectory]
            if length(executed_points) > 1
                lines!(ax, executed_points, 
                    color=agent_color, 
                    linewidth=2, 
                    alpha=0.3
                )
            end
        end

        # 2. Plot committed trajectory from commitment time to current time
        if hasfield(typeof(agent), :committed_trajectory) && !isnothing(agent.committed_trajectory)
            trajectory_points = []
            commitment_time = agent.committed_trajectory.t_committed
            future_time = r3r.scaled_time(model) + 5.0
            
            # Sample trajectory from commitment time to current time
            t_sample = commitment_time:0.1:future_time

            for t in t_sample
                try
                    pos_t = r3r.Gatekeeper.get_position(agent.committed_trajectory, t)
                    if !any(isnan.(pos_t))
                        push!(trajectory_points, Point2f(pos_t[1], pos_t[2]))
                    end
                catch
                    break
                end
            end

            # Plot the nominal trajectory as a dashed line
            trajectory_points_nominal = []
            for t in t_sample
                pt = r3r.Gatekeeper.get_nominal_position(agent.committed_trajectory, t)
                push!(trajectory_points_nominal, Point2f(pt[1], pt[2]))
                continue

                idx = findlast(x -> begin
                    path, start_time = x
                    start_time <= t
                end, agent.committed_trajectory.nominal_trajectory)

                if idx === nothing
                    println("This is getting triggered")
                    break
                    # last_trajectory = agent.committed_trajectory.nominal_trajectory[end][1]
                    # pt = dubins_path_sample(last_trajectory, dubins_path_length(last_trajectory))[2]
                    # push!(trajectory_points_nominal, Point2f(pt[1], pt[2]))
                    # continue
                end

                 # Difference in path start time and current time
                time_along_path = t - agent.committed_trajectory.nominal_trajectory[idx][2]
                pt = dubins_path_sample(agent.committed_trajectory.nominal_trajectory[idx][1], time_along_path)[2]
                push!(trajectory_points_nominal, Point2f(pt[1], pt[2]))
            end
            
            if length(trajectory_points) > 1
                lines!(ax, trajectory_points, 
                    color=agent_color, 
                    linewidth=4, 
                    alpha=0.8
                )
                
                # 2. Label committed trajectory
                if length(trajectory_points) > 10
                    # mid_point = trajectory_points[div(length(trajectory_points), 2)]
                    mid_point = trajectory_points[min(length(trajectory_points), 10)]
                    # text!(ax, mid_point[1] - 0.5, mid_point[2] - 0.5,  
                    #     text=L"\mathcal{T}_%$(agent_idx)^{\text{com}}",
                    #     fontsize=14,
                    #     align=(:center, :center),
                    #     offset=(10, 10)
                    # )
                end
            end

            if length(trajectory_points_nominal) > 1
                lines!(ax, trajectory_points_nominal, 
                    color=agent_color, 
                    linewidth=2, 
                    linestyle=:dash,
                    alpha=0.6
                )
            end
            
            # 5. Plot Rplan circle around commitment point (shaded)
            commitment_pos = r3r.Gatekeeper.get_position(agent.committed_trajectory, commitment_time)
            plan_circle_x = commitment_pos[1] .+ model.Rplan .* cos.(θ)
            plan_circle_y = commitment_pos[2] .+ model.Rplan .* sin.(θ)
            
            # Shaded circle
            poly!(ax, Point2f.(plan_circle_x, plan_circle_y),
                color=(agent_color, 0.2),
                strokecolor=agent_color,
                strokewidth=2
            )
            
            # 4. Label shaded circle (Rplan) - only for first agent to avoid clutter
            # if i == 1
            #     text!(ax, commitment_pos[1] + model.Rplan * 0.7, commitment_pos[2] + model.Rplan * 0.7,
            #         text=L"R^{\text{plan}}",
            #         fontsize=16,
            #         align=(:center, :center)
            #     )
            # end
        end
        
        # 3. Current position of agents
        pos = Point2f(agent.pos[1], agent.pos[2])
        scatter!(ax, [pos], 
            color=agent_color, 
            markersize=15, 
            strokecolor=:black,
            strokewidth=2,
            label="Agent $(agent.id)"
        )
        
        # 1. Label current position
        # text!(ax, agent.pos[1], agent.pos[2] + 0.3,
        #     text=L"p_{%$(agent_idx)}(%$(round(current_time, digits=1)))",
        #     fontsize=14,
        #     align=(:center, :center)
        # )
        
        # Goal state of agents
        goal_pos = Point2f(agent.goal[1], agent.goal[2])
        scatter!(ax, [goal_pos], 
            color=agent_color, 
            marker=:star5, 
            markersize=12, 
            strokecolor=:black, 
            strokewidth=1
        )
        
        # Label goal position with g_i
        # text!(ax, agent.goal[1] + 0.2, agent.goal[2] + 0.2,
        #     text=L"g_{%$(agent_idx)}",
        #     fontsize=14,
        #     align=(:center, :center)
        # )
        
        # 3. Label backup set (goal region)
        backup_center = agent.committed_trajectory.backup_set.orbit_center
        # text!(ax, backup_center[1], backup_center[2],
        #     text=L"\mathcal{C}_%$(agent_idx)",
        #     fontsize=14,
        #     align=(:center, :center)
        # )
        # text!(ax, agent.goal[1], agent.goal[2] - 0.4,
        #     text=L"\mathcal{C}_%$(agent_idx)",
        #     fontsize=14,
        #     align=(:center, :center)
        # )
        
        # Direction arrow showing orientation
        if hasfield(typeof(agent), :theta)
            arrow_length = 0.3
            arrows!(ax, 
                [agent.pos[1]], [agent.pos[2]], 
                [arrow_length * cos(agent.theta)], [arrow_length * sin(agent.theta)],
                color=agent_color, 
                linewidth=3,
                arrowsize=15
            )
        end
        
        # 4. Communication radius circle
        comm_circle_x = agent.pos[1] .+ model.Rcomm .* cos.(θ)
        comm_circle_y = agent.pos[2] .+ model.Rcomm  .* sin.(θ)
        lines!(ax, comm_circle_x, comm_circle_y, 
            color=agent_color, 
            alpha=0.5, 
            linewidth=2, 
            linestyle=:solid
        )
        
        # 5. Label communication radius - only for first agent to avoid clutter
        # if i == 1
        #     text!(ax, agent.pos[1] + model.Rcomm * 1.00, agent.pos[2] + model.Rcomm * 0.5,
        #         text=L"R^{\text{comm}}",
        #         fontsize=16,
        #         align=(:center, :center)
        #     )
        # end
    end
    
    # # Add legend
    # legend_elements = [
    #     MarkerElement(color=:gray, marker=:circle, markersize=15),
    #     MarkerElement(color=:gray, marker=:star5, markersize=12),
    #     LineElement(color=:gray, linestyle=:solid, linewidth=4),
    #     LineElement(color=:gray, linestyle=:solid, linewidth=2, alpha=0.3),
    #     PolyElement(color=(:gray, 0.2), strokecolor=:gray, strokewidth=2),
    #     LineElement(color=:gray, linestyle=:dash, linewidth=2)
    # ]
    
    # legend_labels = [
    #     "Current Position",
    #     "Goal Position", 
    #     "Committed Trajectory",
    #     "Executed Trajectory",
    #     "Planning Radius (Rplan)",
    #     "Communication Radius"
    # ]

    # Legend(fig[1, 2], legend_elements, legend_labels, "Legend", tellheight=false)
    
    # return fig
end

function init_model()
    N_AGENTS = 2

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

    turning_radius = 0.25
    delta = 0.05
    model = r3r.init_dubins_agent_2d_problem(;
            starting_positions=starting_positions, 
            goal_positions=goal_positions,
            n_agents=N_AGENTS, 
            seed=random_seed, 
            delta=delta, 
            Rcomm=rcomm, 
            turning_radius=turning_radius, 
            dim=10.0) 

    return model
end
function make_plot()
    model = init_model()

    fig = Figure(resolution=(900, 400))
    colgap!(fig.layout, 0)
    rowgap!(fig.layout, 0)

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 1.0
    agent_df, model_df = run!(model, till_done)

    # Print Initial State
    pretty_plot_system(model, Axis(fig[1, 1], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 100.0
    agent_df, model_df = run!(model, till_done)
    pretty_plot_system(model, Axis(fig[1, 2], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 100.0
    agent_df, model_df = run!(model, till_done)
    pretty_plot_system(model, Axis(fig[1, 3], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 200.0
    agent_df, model_df = run!(model, till_done)
    pretty_plot_system(model, Axis(fig[2, 1], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 200.0
    agent_df, model_df = run!(model, till_done)
    pretty_plot_system(model, Axis(fig[2, 2], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    till_done = (model_in, t) -> nagents(model_in) == 0  || t > 200.0
    agent_df, model_df = run!(model, till_done)
    pretty_plot_system(model, Axis(fig[2, 3], alignmode = Mixed(
        left = Makie.Protrusion(0),
        right = Makie.Protrusion(0),
        bottom = Makie.Protrusion(0),
        top = Makie.Protrusion(0)
    )))

    resize_to_layout!(fig)

     # # Add legend
    legend_elements = [
        MarkerElement(color=:gray, marker=:circle, markersize=15),
        MarkerElement(color=:gray, marker=:star5, markersize=12),
        LineElement(color=:gray, linestyle=:solid, linewidth=4),
        LineElement(color=:gray, linestyle=:dash, linewidth=2, alpha=0.3),
        PolyElement(color=(:gray, 0.2), strokecolor=:gray, strokewidth=2),
        LineElement(color=:gray, linestyle=:solid, linewidth=2)
    ]
    
    legend_labels = [
        "Current Position",
        "Goal Position", 
        "Committed Trajectory",
        "Nominal Trajectory",
        "Planning Radius (Rplan)",
        "Communication Radius"
    ]

    leg = Legend(fig[3, 1:3], legend_elements, legend_labels,  tellheight=false, orientation= :horizontal, nbanks=2)
    leg.framevisible = false


    save("images/explanatory/combined_figure.pdf", fig)
    return fig
end