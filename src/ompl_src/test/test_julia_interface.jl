module OMPLSTRRTTest

# Julia test file for OMPL ST-RRT* planner
# This demonstrates how to use the C interface with ccall

# Path to the compiled shared library
# Adjust this path based on your build output
# const LIB_PATH = "../build/libompl_st_rrt.so"  # Linux
const LIB_PATH = "../build/libompl_src.dylib"  # macOS
# const LIB_PATH = "../build/ompl_st_rrt.dll"  # Windows

# Initialize the planner
function init_planner(space_dim::Int, delta::Float32, v_max::Float32, t_weight::Float32)
    ccall((:init, LIB_PATH), Cvoid,
        (Cint, Cfloat, Cfloat, Cfloat),
        space_dim, delta, v_max, t_weight)
end

OBSTACLES = []
function isValid(time::Float64, x::Float64, y::Float64)::Cint
    for obs in OBSTACLES
        (ox, oy, is_valid) = obs(time)
        if is_valid
            dist_sq = (x - ox)^2 + (y - oy)^2
            if dist_sq < 5.0  # Assume obstacle radius is √5.0
                return Cint(0)
            end
        end
    end

    return Cint(1)
end

# Simple planning function without obstacles
function plan_simple(current_state::Vector{Float64}, goal_state::Vector{Float64}, max_path_length::Int=100)
    # Pre-allocate output arrays
    path_out = zeros(Float64, max_path_length * 3)  # x, y, t for each waypoint
    actual_path_length = Ref{Cint}(0)

    # Call the C function
    success = ccall((:plan_strrt_simple_julia, LIB_PATH), Cint,
        (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Cint, Ptr{Cint}),
        current_state, goal_state, path_out, max_path_length, actual_path_length)

    if success == 1 && actual_path_length[] > 0
        # Extract the actual kpath
        path_length = actual_path_length[]
        path = Vector{Vector{Float64}}()

        for i in 1:path_length
            idx = (i - 1) * 3 + 1
            waypoint = [path_out[idx], path_out[idx+1], path_out[idx+2]]
            push!(path, waypoint)
        end

        return path
    else
        return nothing  # Planning failed
    end
end

# Planning function with dynamic obstacles
function plan_with_obstacles(current_state::Vector{Float64}, goal_state::Vector{Float64},
    obstacle_functions::Vector{Function}, max_path_length::Int=100)

    # Clear previous obstacles and add new ones
    empty!(OBSTACLES)
    push!(OBSTACLES, obstacle_functions...)

    # Create C function pointer for validity checking
    c_is_valid = @cfunction(isValid, Cint, (Cdouble, Cdouble, Cdouble))

    # Pre-allocate output arrays
    path_out = zeros(Float64, max_path_length * 3)  # x, y, t for each waypoint
    actual_path_length = Ref{Cint}(0)

    # Call the C function with correct signature
    success = ccall((:plan_strrt_julia, LIB_PATH), Cint,
        (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cvoid}, Ptr{Cdouble}, Cint, Ptr{Cint}),
        current_state, goal_state, c_is_valid, path_out, max_path_length, actual_path_length)

    if success == 1 && actual_path_length[] > 0
        # Extract the actual path
        path_length = actual_path_length[]
        path = Vector{Vector{Float64}}()

        for i in 1:path_length
            idx = (i - 1) * 3 + 1
            waypoint = [path_out[idx], path_out[idx+1], path_out[idx+2]]
            push!(path, waypoint)
        end

        return path
    else
        return nothing  # Planning failed
    end
end

# Example usage and test
function test_planning()
    println("Testing OMPL ST-RRT* Julia interface...")

    # Initialize planner
    space_dim = 100
    delta = 1.0f0
    v_max = 1.0f0
    t_weight = 0.5f0

    println("Initializing planner...")
    init_planner(space_dim, delta, v_max, t_weight)

    # Test simple planning
    current_state = [10.0, 10.0, 0.0]  # [x, y, t]
    goal_state = [80.0, 80.0]          # [x, y]

    println("Planning from $current_state to $goal_state...")

    # path = plan_simple(current_state, goal_state)

    # if path !== nothing
    #     println("Planning successful! Found path with $(length(path)) waypoints:")
    #     for (i, waypoint) in enumerate(path)
    #         println("  Waypoint $i: x=$(waypoint[1]), y=$(waypoint[2]), t=$(waypoint[3])")
    #     end
    # else
    #     println("Planning failed!")
    # end

    # println("Testing with obstacles...")

    obs1 = create_circular_obstacle(50.0, 50.0, 10.0)
    obs2 = create_moving_obstacle(20.0, 20.0, 0.5, 0.5)
    obstacles = [obs1, obs2]

    path = plan_with_obstacles(current_state, goal_state, obstacles)

    if path !== nothing
        println("Planning successful! Found path with $(length(path)) waypoints:")
        for (i, waypoint) in enumerate(path)
            println("  Waypoint $i: x=$(waypoint[1]), y=$(waypoint[2]), t=$(waypoint[3])")
        end
        return true
    else
        println("Planning failed!")
        return false
    end

end

# Example of how to define obstacle functions for future use
function create_circular_obstacle(center_x::Float64, center_y::Float64, radius::Float64)
    return function (time::Float64)
        # Static obstacle - position doesn't change with time
        return (center_x, center_y, true)  # (x, y, valid)
    end
end

function create_moving_obstacle(start_x::Float64, start_y::Float64,
    velocity_x::Float64, velocity_y::Float64)
    return function (time::Float64)
        # Moving obstacle
        x = start_x + velocity_x * time
        y = start_y + velocity_y * time
        valid = (x >= 0 && x <= 100 && y >= 0 && y <= 100)  # Within bounds
        return (x, y, valid)
    end
end



# function obstacle_wrapper(obs_func::Function)
#     function do_obs(time::Cfloat, x::Ptr{Cdouble}, y::Ptr{Cdouble}, valid::Ptr{Cint})
#         (ox, oy, is_valid) = obs_func(time)
#         unsafe_store!(x, ox)
#         unsafe_store!(y, oy)
#         unsafe_store!(valid, is_valid ? 1 : 0)
#         return
#     end

#     return @cfunction(do_obs, Cvoid, (Cfloat, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cint}))
# end

# Run the test if this file is executed directly
if abspath(PROGRAM_FILE) == @__FILE__
    try
        success = test_planning()
        exit(success ? 0 : 1)
    catch e
        println("Error running test: $e")
        exit(1)
    end
end

end # module OMPLSTRRTTest

using .OMPLSTRRTTest