

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

    model = ipopt_baseline_model(;starting_positions=starting_positions, goal_positions=goal_positions,n_agents=n_agents, seed=random_seed, delta=delta, Rcomm=Rcomm, Rgoal=Rgoal,turning_radius=turning_radius, dim=5.0)

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

function run_randomized_nmpc_trial(n_agent, r_comm, random_seed, params)
    delta = get(params, :delta, 0.5)
    Rgoal = get(params, :Rgoal, 0.5)
    turning_radius = get(params, :turning_radius, 0.25)

    model = ipopt_baseline_model(;n_agents=n_agent, seed=random_seed, delta=delta, Rcomm=r_comm, Rgoal=Rgoal,turning_radius=turning_radius, dim=20.0)

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

using Printf

"""
    results_to_latex(results_comb;
        percent=false,
        decimals=2,
        bold_row_max=true,
        caption="Success rates",
        label="tab:success_rates",
        sort_rows=true,
        sort_cols=true,
        file::Union{Nothing,String}=nothing,
        booktabs=true)

Convert a collection of tuples `(n_agents, r_comm, success_rate)` into a LaTeX tabular string.

Arguments
- results_comb :: AbstractVector{<:Tuple}: Each element like (Int, Real, Real).
Keyword options
- percent: if true, multiply rates by 100 and append %.
- decimals: number of decimal places (or after % if percent=true).
- bold_row_max: make the maximum entry in each row \\textbf{...}.
- caption / label: LaTeX caption & label (wraps in table env).
- sort_rows / sort_cols: control ordering of unique n_agents and r_comm.
- file: if a path string, also writes the LaTeX to that file.
- booktabs: use \\toprule/\\midrule/\\bottomrule (requires \\usepackage{booktabs}).

Returns
- String containing the LaTeX code.
"""
function results_to_latex(results_comb;
    percent=false,
    decimals=2,
    bold_row_max=true,
    caption="Success rates",
    label="tab:success_rates",
    sort_rows=true,
    sort_cols=true,
    file::Union{Nothing,String}=nothing,
    booktabs=true)

    # Extract unique row / column keys
    n_vals = collect(unique(ntuple -> ntuple[1], results_comb))
    r_vals = collect(unique(ntuple -> ntuple[2], results_comb))
    sort_rows && sort!(n_vals)
    sort_cols && sort!(r_vals)

    # Build lookup dict
    data = Dict{Tuple{Int,Float64},Float64}()
    for (n, r, s) in results_comb
        data[(n, Float64(r))] = Float64(s)
    end

    fmt = percent ?
        (x -> @sprintf("%.*f\\%%", decimals, 100*x)) :
        (x -> @sprintf("%.*f", decimals, x))

    # Start assembling lines
    lines = String[]
    push!(lines, "\\begin{table}[t]")
    push!(lines, "  \\centering")
    push!(lines, "  \\caption{$caption}")
    push!(lines, "  \\label{$label}")
    colspec = "c" * " " * repeat("r", length(r_vals))  # first col (n) then each r_comm
    push!(lines, booktabs ? "  \\begin{tabular}{$colspec}" : "  \\begin{tabular}{$colspec}")
    booktabs && push!(lines, "    \\toprule")

    # Header
    # Use escaped dollar signs so Julia does not attempt string interpolation
    header = ["\$N\$"]
    append!(header, ["\$r_{\\text{comm}} = $(r)\$" for r in r_vals])
    push!(lines, "    " * join(header, " & ") * " \\\\")
    booktabs && push!(lines, "    \\midrule")

    # Rows
    for n in n_vals
        row_values = [get(data, (n, Float64(r)), NaN) for r in r_vals]
        # Determine bold target(s)
        max_indices = Int[]
        if bold_row_max
            finite_vals = filter(!isnan, row_values)
            if !isempty(finite_vals)
                m = maximum(finite_vals)
                for (j,v) in enumerate(row_values)
                    if !isnan(v) && isapprox(v, m; atol=10.0^(-decimals-2))
                        push!(max_indices, j)
                    end
                end
            end
        end
        cells = String[]
        for (j,v) in enumerate(row_values)
            if isnan(v)
                push!(cells, "--")
            else
                sv = fmt(v)
                if j in max_indices
                    sv = "\\textbf{$sv}"
                end
                push!(cells, sv)
            end
        end
        push!(lines, "    " * string(n) * " & " * join(cells, " & ") * " \\\\")
    end

    booktabs && push!(lines, "    \\bottomrule")
    push!(lines, "  \\end{tabular}")
    push!(lines, "\\end{table}")

    table_str = join(lines, '\n')

    if file !== nothing
        open(file, "w") do io
            write(io, table_str * "\\n")
        end
    end

    return table_str
end
