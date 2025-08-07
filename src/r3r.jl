module r3r

include("common/R3RCommon.jl")
using .R3RCommon
# export scaled_time, exact_nearby_agents, comms_radius, collision_radius, squared_dist

include("gatekeeper.jl")
using .Gatekeeper

# include("double_integrators_gatekeeper.jl")
# using .Gatekeeper2D

# include("2d_double_integrators.jl")
# using .double_integrators_2d

# include("2d_path_tracker.jl")
# using .PathTracker2D
# export init_2d_path_tracker_problem

include("agents/dubins_agent_2d_model.jl")
using .DubinsAgent2DModel
export init_dubins_agent_2d_problem

end # module r3r
