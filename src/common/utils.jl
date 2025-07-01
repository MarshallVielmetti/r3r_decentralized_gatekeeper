using Agents, LinearAlgebra

function scaled_time(model)
    return abmtime(model) * model.dt
end

function exact_nearby_agents(agent, model, radius)
    # Get the base iterator from nearby_agents
    base_iterator = nearby_agents(agent, model, radius, search=:exact)

    # Return a filtered iterator  to only get agents in the network
    return Iterators.filter(neighbor_agent -> neighbor_agent.in_network, base_iterator)
end

function comms_radius(agent, model)
    return exact_nearby_agents(agent, model, model.Rcomm)
end

function collision_radius(agent, model)
    return exact_nearby_agents(agent, model, model.delta)
end

function in_collision(agent, model)
    for neighbor in collision_radius(agent, model)
        return true
    end
    return false
end

function squared_dist(v1::AbstractVector{Float64}, v2::AbstractVector{Float64})::Float64
    return sum((v1 .- v2) .^ 2)
end

function solve_quadratic(a::Float64, b::Float64, c::Float64)
    discr = b^2 - 4 * a * c
    if discr < 0
        return nothing # no real solutions
    else
        return (-b + sqrt(discr)) / (2 * a), (-b - sqrt(discr)) / (2 * a)
    end
end

"""
    compute_intersection_point(x1, x2, pos, r)

Computes the point of intersection between a line segment defined by points
`x1` and `x2`, and a circle centered at `pos` with radius `r`.

Only returns the intersection point if it exists and is within the segment.
"""
function compute_intersection_point(x1, x2, pos, r)
    # Vector from x1 to x2
    d = x2 - x1
    # Vector from x1 to pos
    f = x1 - pos  # Fixed: should be x1 - pos, not pos - x1

    a = dot(d, d)
    b = 2 * dot(f, d)
    c = dot(f, f) - r^2

    # Solve the quadratic equation
    solutions = solve_quadratic(a, b, c)

    if solutions === nothing
        return nothing # No intersection
    end

    t1, t2 = solutions

    if 0 <= t1 <= 1
        return x1 + t1 * d
    end

    if 0 <= t2 <= 1
        return x1 + t2 * d
    end

    return nothing
end