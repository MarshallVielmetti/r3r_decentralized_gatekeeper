module STRRT_STAR

using StaticArrays

const LIB_PATH = "./ompl_src/build/libompl_src.dylib"

export init_planner, plan_nominal

function init_planner(space_dim::Int, delta::Float32, v_max::Float32, t_weight::Float32)
    ccall((:init, LIB_PATH), Cvoid,
        (Cint, Cfloat, Cfloat, Cfloat),
        space_dim, delta, v_max, t_weight)
end


# OBSTACLES::Vector{Function} = Function[]
# function isValid(time::Float64, x::Float64, y::Float64)::Cint
#     for obs in OBSTACLES
#         (ox, oy, is_valid) = obs(time)
#         if is_valid
#             dist_sq = (x - ox)^2 + (y - oy)^2
#             if dist_sq < 5.0  # Assume obstacle radius is √5.0
#                 return Cint(0)
#             end
#         end
#     end

#     return Cint(1)
# end

# function make_valid_function(obstacle_functions::Vector{Function})
#     global OBSTACLES = obstacle_functions
#     return @cfunction(isValid, Cint, (Cdouble, Cdouble, Cdouble))
# end

function plan_nominal(current_state::SVector{3,Float64}, goal_state::SVector{2,Float64}, valid_function_ptr, max_path_length::Int=100)
    # Create the C-compatible arrays
    path_out = zeros(Float64, 3 * max_path_length) # x,y,t for each waypoint
    actual_path_length = Ref{Cint}(0)

    # Call the C function with correct signature
    success = ccall((:plan_strrt_julia, LIB_PATH), Cint,
        (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cvoid}, Ptr{Cdouble}, Cint, Ptr{Cint}),
        current_state, goal_state, valid_function_ptr, path_out, max_path_length, actual_path_length)

    # Store the value
    actual_path_length = actual_path_length[]

    # Check for success
    if success == 0 || actual_path_length == 0
        return nothing  # Planning failed
    end

    # Return a view of the path as a matrix (3, N) without copying
    return @view reshape(path_out, 3, max_path_length)[:, 1:actual_path_length]
end

end