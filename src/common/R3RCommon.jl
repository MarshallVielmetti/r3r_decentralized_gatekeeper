module R3RCommon

include("types.jl")
include("utils.jl")

# Trajectory Types
export AbstractCompositeTrajectory, CompositeTrajectory, PathFollowerCompositeTrajectory
export DoubleIntegrator2D, PathFollower2D

# Agents.jl Utilities
export scaled_time, exact_nearby_agents, comms_radius, collision_radius, in_collision

# Math Utilities
export squared_dist, solve_quadratic, compute_intersection_point

end