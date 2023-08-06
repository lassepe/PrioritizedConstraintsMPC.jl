struct Solver{T<:ParametricMCPs.ParametricMCP}
    parametric_mcp::T
end

"""
This constructor symbolically evaluates the dynamics and constraints to set up the KKT conditions
as a mixed complementarity problem (MCP) of the form `f(z, θ) ⟂ lb ≤ z ≤ ub` that we can solve with `ParametricMCPs.jl`.
"""
function Solver(dynamics, environment, objective; planning_horizon = 10)
    state_dimension = TGB.state_dim(dynamics)
    control_dimension = TGB.control_dim(dynamics)
    goal_parameter_dimension = 2

    # symbolic variables for the primal decision variables: states and inputs at every time step
    states = Symbolics.variables(:x, 1:state_dimension, 1:planning_horizon) |> eachcol |> collect
    controls =
        Symbolics.variables(:u, 1:control_dimension, 1:planning_horizon) |> eachcol |> collect

    initial_state = Symbolics.variables(:x, 1:state_dimension)
    goal_position = Symbolics.variables(:x, 1:goal_parameter_dimension)

    # TASK 2.1: Set up
    #   - the dynamics and
    #   - initial state constraints
    #   as equality constraints.
    # Tip 1: `equality_constraints` will be a vector-valued expression which is zero when the constraints are satisfied. I.e., we enforce `equality_constraints == 0`
    equality_constraints = nothing
    # Tip 2: The dynamics can be called as `dynamics(state, input, time)` and returns the next state.
    # More information on the dynamics interface here: https://github.com/lassepe/TrajectoryGamesBase.jl/blob/a15c15d31474c5da4e826418651f9787cc09383a/src/dynamics.jl#L19
    # This is a function which takes a `state` (at a single time) and returns a vector of
    # numbers which are non-negative when the state is inside the environment.
    environment_constraints = TGB.get_constraints(environment)
    # The vector-valued lower and upper bounds for the state
    state_min, state_max = TGB.state_bounds(dynamics)
    # The vector-valued lower and upper bounds for the control inputs
    control_min, control_max = TGB.control_bounds(dynamics)
    # TASK 2.2: Set up the environment constraints as inequality as well as the state and input bounds
    # as inequality constraints.
    # Tip 1: `inequality_constraints` will be a vector-valued expression which is *non-negative* when the constraints are satisfied. I.e., we enforce `inequality_constraints >= 0`
    # Tip 2: use the building blows above.
    inequality_contraints = nothing

    # TASK 2.3: derive dual variables for both equality and inequality constraints
    λ = nothing
    μ = nothing

    # TASK 2.4: Set up the Lagrangian
    L = nothing

    # TASK 2.5: set up the KKT conditions as a mixed complementarity problem (MCP)
    # Tip 1: you can use `Symbolics.gradient` to compute the gradient of the Lagrangian
    # Tip 2: `f` will be one long vector of symbolic expressions, `ub` and `lb` have the same
    # dimension as `f` but are vectors of extended real numbers (not symbolic variables).
    # Tip 3: if you are not familiar with MCPs, read up on them here: https://en.wikipedia.org/wiki/Mixed_complementarity_problem and in
    # "Finite-Dimensional Variational Inequalities and Complementarity Problems" Volume by Facchinei and Pang.
    f = nothing
    lb = nothing
    ub = nothing

    # TASK 2.6: Once you've implemented all steps above
    #z = [stack(states)[:], stack(controls)[:], λ, μ]
    #θ = [initial_state; goal_position]
    #parametric_mc = ParametricMCPs.ParametricMCP(f, z, θ, lb, ub; compute_sensitivities = false)
    #Solver(parametric_mcp)
    nothing
end

function solve_trajectory_optimization(
    solver::Solver,
    initial_state,
    goal_position;
    initial_guess = nothing,
)
    θ = [initial_state; goal_position]
    ParametricMCPs.solve(solver.parametric_mcp, θ; initial_guess)
end
