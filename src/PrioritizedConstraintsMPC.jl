module PrioritizedConstraintsMPC

using GLMakie: GLMakie
using Makie: Makie
using Random: Random
using ParametricMCPs: ParametricMCPs
using LazySets: sample
using TrajectoryGamesBase: TrajectoryGamesBase as TGB
using TrajectoryGamesExamples: TrajectoryGamesExamples as TGE
using Symbolics: Symbolics
using FileIO: load
using LinearAlgebra: norm
using LinearAlgebra

export run_demo

# this is the main file where you need to make your changes:
include("solver.jl")
# you don't need to change anything here but you can take a look to get some intuition about how the
# constraints are set up
include("validation.jl")
# you do not need to change anything in this file (unless you want to improve the visualization for
# debugging purposes)
include("simulation.jl")

function run_demo(;
    rng = Random.default_rng(),
    environment = TGB.PolygonEnvironment(),
    dynamics = TGE.planar_double_integrator(;
        state_bounds = (; lb = [-Inf, -Inf, -0.5, -0.75], ub = [Inf, Inf, 0.75, 0.75]),
        control_bounds = (; lb = [-1.0, -1.0], ub = [1.0, 1.0]),
    ),
    initial_state = let
        initial_position = sample(environment.set; rng)
        initial_velocity = [0.0, 0.0]
        [initial_position; initial_velocity]
    end,
    goal_position = sample(environment.set; rng),
)
    function objective(states, controls)
        sum(zip(states, controls)) do (state, input)
            # TASK 1: Implement a quadratic cost function that penalizes the distance to the goal position.
            # Also add a small penalty on the input to regularize the solution.
            (state-[goal_position;0;0])'*Diagonal([1,1,1,1])*(state-[goal_position;0;0]) + 0.1*input'*Diagonal([1,1])*input
            # println(state)
            # println(input)
            # 0.0
        end
    end

    # TASK 2: Implement the solver constructor. More details on sub-steps in `src/solver.jl`
    solver = Solver(dynamics, environment, objective)
    strategy = function (state, time)
        # TASK 3: After implementing `solve_trajectory_optimization`,
        # extract the first input from the solution and return it from this function (instead of `[0.0, 0.0]` below)
        solution = solve_trajectory_optimization(solver, state, goal_position)
        println(solution.z[41:42])
        solution.z[41:42]
    end

    simulate!(strategy, dynamics, environment, initial_state, goal_position;)
end

end # module PrioritizedConstraintsMPC
