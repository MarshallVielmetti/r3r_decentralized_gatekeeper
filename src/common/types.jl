using Agents, StaticArrays

## TRAJECTORY TYPES ##
abstract type AbstractCompositeTrajectory end

struct CompositeTrajectory{TN,TTN,TB,TBS} <: AbstractCompositeTrajectory
    t_committed::Float64
    t_switch::Float64
    t_bak::Float64
    nominal_trajectory::TN
    tracked_nominal_trajectory::TTN
    backup_trajectory::TB
    backup_set::TBS
end

struct PathFollowerCompositeTrajectory{TN,TBS} <: AbstractCompositeTrajectory
    t_committed::Float64
    t_switch::Float64
    t_bak::Float64
    nominal_trajectory::TN
    backup_set::TBS
end

struct DubinsCompositeTrajectory{TN,TBS} <: AbstractCompositeTrajectory
    t_committed::Float64
    t_switch::Float64
    t_bak::Float64
    nominal_trajectory::TN
    backup_set::TBS
end

## AGENT TYPES ##

# abstract type AbstractGatekeeperAgent2D end

@agent struct DoubleIntegrator2D(ContinuousAgent{2,Float64})
    # has pos and vel built into the agent type
    in_network::Bool # Flag to indicate whether or not the agent has joined the network yet
    failed_last_replan::Bool # Flag to indicate if the agent failed to replan at last attempt
    my_neighbor_replanned::Bool # Flag to indicate if an agent's neighbor has replanned at the current time step
    goal::SVector{2,Float64} # Goal Position
    committed_trajectory::Union{Nothing,CompositeTrajectory{TN,TB}} where {TN,TB} # Composite trajectory for the agent or nothing
end

@agent struct PathFollower2D(ContinuousAgent{2,Float64})
    # has pos and vel built into the agent type
    in_network::Bool # Flag to indicate whether or not the agent has joined the network yet
    failed_last_replan::Bool # Flag to indicate if the agent failed to replan at last attempt
    just_replanned::Bool # Flag to indicate if an agent's neighbor has replanned at the current time step
    goal::SVector{2,Float64} # Goal Position
    committed_trajectory::Union{Nothing,PathFollowerCompositeTrajectory{TN,TBS}} where {TN,TBS} # Composite trajectory for the agent or nothing
end


"""
    DubinsAgent2D
Notes:
It is in R3 because the state is x,y,θ
"""
@Agent struct DubinsAgent2D(ContinuousAgent{3,Float64})
    # has pos and vel built into the agent type
    in_network::Bool # Flag to indicate whether or not the agent has joined the network yet
    failed_last_replan::Bool # Flag to indicate if the agent failed to replan at last attempt
    just_replanned::Bool # Flag to indicate if an agent's neighbor has replanned at the current time step
    goal::SVector{3,Float64} # Goal Position
    committed_trajectory::Union{Nothing,PathFollowerCompositeTrajectory{TN,TBS}} where {TN,TBS} # Composite trajectory for the agent or nothing
    turning_radius::Float64 # Minimum turning radius for Dubins dynamics
end