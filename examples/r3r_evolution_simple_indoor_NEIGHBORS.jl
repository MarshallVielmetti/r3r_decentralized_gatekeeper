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
rcomm = 16.0

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

# Advance simulation between snapshots
till_done = (model_in, t) -> nagents(model_in) == 0  || t > 300.0
agent_df, model_df = run!(model, till_done; adata=[:count_neighbors_at_replan, :time_to_replan, :failed_last_replan])

# save("pretty_system_summary.pdf", fig)
agent_df_filter = filter(row -> row.time_to_replan > 0.0, agent_df)
print("avg. time to replan: ", mean(agent_df_filter.time_to_replan), "\n")
print("avg. number of neighbors at replan: ", mean(agent_df_filter.count_neighbors_at_replan), "\n")

# # Scatter plot: neighbors at replan vs. time to replan
# if !isempty(agent_df_filter)
#     fig_neighbors = Figure(resolution=(900, 600))
#     ax_neighbors = Axis(fig_neighbors[1, 1];
#         xlabel = "# Neighbors at replan",
#         ylabel = "Time to replan (s)",
#         title = "Replan latency vs. neighborhood size",
#         xgridvisible = false,
#         ygridvisible = false
#     )

#     scatter!(ax_neighbors,
#         agent_df_filter.count_neighbors_at_replan,
#         agent_df_filter.time_to_replan;
#         color = ColorSchemes.tab10.colors[2],
#         markersize = 12,
#         strokecolor = :black,
#         strokewidth = 0.4,
#         transparency = true
#     )

#     display(fig_neighbors)

#     grouped_stats = combine(groupby(agent_df_filter, :count_neighbors_at_replan)) do sdf
#         vals = sdf.time_to_replan
#         mean_val = mean(vals)
#         std_val = length(vals) > 1 ? std(vals; corrected=false) : 0.0
#         (; count_neighbors_at_replan = first(sdf.count_neighbors_at_replan),
#            mean_time = mean_val,
#            std_time = std_val)
#     end
#     sort!(grouped_stats, :count_neighbors_at_replan)

#     fig_line = Figure(resolution=(900, 600))
#     ax_line = Axis(fig_line[1, 1];
#         xlabel = "# Neighbors at replan",
#         ylabel = "Average time to replan (s)",
#         title = "Average replan latency with variability",
#         xgridvisible = false,
#         ygridvisible = false
#     )

#     xvals = grouped_stats.count_neighbors_at_replan
#     yvals = grouped_stats.mean_time
#     err = grouped_stats.std_time

#     errorbars!(ax_line, xvals, yvals, err, err; color = ColorSchemes.tab10.colors[3], whiskerwidth = 10)
#     lines!(ax_line, xvals, yvals; color = ColorSchemes.tab10.colors[3], linewidth = 3)

#     scatter!(ax_line, xvals, yvals; color = ColorSchemes.tab10.colors[3], markersize = 10)

#     display(fig_line)
# else
#     @warn "agent_df_filter is empty; skipping scatter plot"
# end

