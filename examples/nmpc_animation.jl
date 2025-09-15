include("examples/ipopt_baseline_utils.jl")

using Agents, DataFrames, Statistics, Random
using CairoMakie, Colors
using r3r

N_AGENTS = 8

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

rcomm = 3.0
turning_radius = 0.25
delta = 0.5
rgoal = 0.5

model = ipopt_baseline_model(;starting_positions=starting_positions, goal_positions=goal_positions,n_agents=N_AGENTS, seed=random_seed, delta=delta, Rcomm=rcomm, Rgoal=rgoal,turning_radius=turning_radius, dim=10.0)

# Initialize data structures to store trajectories over time using agent IDs
max_steps = 200
all_agent_ids = [agent.id for agent in allagents(model)]  # Store initial agent IDs
all_positions = Dict{Int, Vector{Tuple{Float64, Float64}}}()  # Map from agent ID to positions
all_goals = Dict{Int, Tuple{Float64, Float64}}()  # Map from agent ID to goals

# Initialize position vectors for each agent
for agent in allagents(model)
    all_positions[agent.id] = Vector{Tuple{Float64, Float64}}()
    all_goals[agent.id] = (agent.goal[1], agent.goal[2])
end

# Step through the simulation
for step in 1:max_steps
    # Store current positions for all agents still in simulation
    current_agent_ids = [agent.id for agent in allagents(model)]
    
    for agent in allagents(model)
        push!(all_positions[agent.id], (agent.pos[1], agent.pos[2]))
    end
    
    # For agents that have been removed, add their last known position
    for agent_id in all_agent_ids
        if !(agent_id in current_agent_ids) && !isempty(all_positions[agent_id])
            # Agent was removed, repeat last position
            last_pos = all_positions[agent_id][end]
            push!(all_positions[agent_id], last_pos)
        end
    end
    
    # Step the model
    step!(model)
    
    # Break if all agents have reached their goals
    if isempty(allagents(model))
        println("All agents reached their goals at step $step")
        break
    end
end

# Create animation
fig = Figure(resolution = (800, 800))
ax = Axis(fig[1, 1], xlabel = "x", ylabel = "y", title = "Multi-Agent MPC Simulation")

# Set axis limits based on world dimensions
xlims!(ax, 0, model.dims[1])
ylims!(ax, 0, model.dims[2])

# Colors for different agents
colors = distinguishable_colors(length(all_agent_ids))
color_map = Dict(zip(all_agent_ids, colors))

# Find maximum number of steps recorded
max_recorded_steps = maximum(length(positions) for positions in values(all_positions))

# Record animation
record(fig, "mpc_animation.gif", 1:max_recorded_steps, framerate = 10) do frame
    empty!(ax)
    
    # Plot trajectories up to current frame for each agent
    for (i, agent_id) in enumerate(all_agent_ids)
        positions = all_positions[agent_id]
        if frame <= length(positions)
            # Plot trajectory
            if frame > 1
                traj_x = [pos[1] for pos in positions[1:frame]]
                traj_y = [pos[2] for pos in positions[1:frame]]
                lines!(ax, traj_x, traj_y, color = color_map[agent_id], alpha = 0.5, linewidth = 2)
            end
            
            # Plot current position
            scatter!(ax, [positions[frame][1]], [positions[frame][2]], 
                    color = color_map[agent_id], markersize = 15, label = "Agent $agent_id")
            
            # Plot goal
            goal = all_goals[agent_id]
            scatter!(ax, [goal[1]], [goal[2]], 
                    color = color_map[agent_id], marker = :star5, markersize = 20, alpha = 0.7)
        end
    end
    
    # Update title with current step
    ax.title = "Multi-Agent MPC Simulation - Step $frame"
end

println("Animation saved as mpc_animation.gif")

# Calculate minimum distances at each time step using the ID-based data
min_distances = Float64[]
max_recorded_steps = maximum(length(positions) for positions in values(all_positions))
time_steps = 1:max_recorded_steps

for step in time_steps
    min_dist = Inf
    agent_ids = collect(keys(all_positions))
    
    # Check all pairs of agents at this time step
    for i in 1:length(agent_ids)
        for j in i+1:length(agent_ids)
            id_i, id_j = agent_ids[i], agent_ids[j]
            
            if step <= length(all_positions[id_i]) && step <= length(all_positions[id_j])
                pos_i = all_positions[id_i][step]
                pos_j = all_positions[id_j][step]
                
                # Skip if agents are at the same position (both reached goal)
                if pos_i != pos_j
                    dist = sqrt((pos_i[1] - pos_j[1])^2 + (pos_i[2] - pos_j[2])^2)
                    min_dist = min(min_dist, dist)
                end
            end
        end
    end
    
    push!(min_distances, min_dist == Inf ? NaN : min_dist)
end

# Create the line plot
fig_dist = Figure(resolution = (800, 400))
ax_dist = Axis(fig_dist[1, 1], 
    xlabel = "Time Step", 
    ylabel = "Minimum Distance", 
    title = "Minimum Inter-Agent Distance Over Time"
)

# Plot minimum distance (filter out NaN values for better visualization)
valid_indices = .!isnan.(min_distances)
if any(valid_indices)
    lines!(ax_dist, time_steps[valid_indices], min_distances[valid_indices], 
           color = :blue, linewidth = 2, label = "Min Distance")
end

# Add horizontal line for collision radius (delta)
if @isdefined(model)
    hlines!(ax_dist, [model.delta], color = :red, linestyle = :dash, linewidth = 2, label = "Collision Radius (δ)")
else
    # Use a typical value if model not available
    hlines!(ax_dist, [0.25], color = :red, linestyle = :dash, linewidth = 2, label = "Collision Radius (δ≈0.25)")
end

# Add legend
axislegend(ax_dist)

# Set y-axis to start from 0 for better visualization
valid_distances = filter(!isnan, min_distances)
if !isempty(valid_distances)
    ylims!(ax_dist, 0, maximum(valid_distances) * 1.1)
end

# Save the distance plot as PNG
save("min_distance_plot.png", fig_dist)
println("Distance plot saved as min_distance_plot.png")