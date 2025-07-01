using Pkg;
Pkg.activate("..")

using Agents

include("r3r.jl")
using .r3r

include("plot_utils.jl")

agent_data = [:pos, :in_network]

function new_model()
    include("r3r.jl")
    return r3r.init_2d_path_tracker_problem(n_agents=10, seed=123123, delta=4.0)
end

function circle_model(n_agents, circle_radius=30.0)
    include("r3r.jl")

    # Create a set of n_agents points on a circle of radius circle_radius, centered at (50, 50)
    starting_positions = [SVector{2,Float64}(circle_radius * cos(2 * π * i / n_agents) + 50,
        circle_radius * sin(2 * π * i / n_agents) + 50) for i in 0:n_agents-1]

    # SEt the goal positions to be opposite the starting positions
    goal_positions = [starting_positions[Int((i + n_agents / 2) % n_agents + 1)] for i in 1:n_agents]

    return r3r.init_2d_path_tracker_problem(
        n_agents=n_agents,
        delta=4.0,
        Rcomm=16.0,
        Rgoal=0.5,
        dt=0.1,
        starting_positions=starting_positions,
        goal_positions=goal_positions,
    )
end

function run_model(model, duration=200)
    agent_df, model_df = run!(model, duration; adata=agent_data)
    return agent_df
end

function plot(agent_df, model)
    PlotUtils.animate_df(agent_df, model)
end
