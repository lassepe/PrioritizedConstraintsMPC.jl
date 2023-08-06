struct Validator{T1,T2,T3}
    environment::T1
    dynamics::T2
    initial_state::T3
    tolerance::Float64
    verbose::Bool
    previous_state::T3
    previous_input::T3

    function Validator(;
        environment::T1,
        dynamics::T2,
        initial_state::T3,
        tolerance = 1e-3,
        verbose = true,
    ) where {T1,T2,T3}
        new{T1,T2,T3}(
            environment,
            dynamics,
            initial_state,
            tolerance,
            verbose,
            initial_state,
            zeros(TGB.control_dim(dynamics)),
        )
    end
end

function validate_step!(validator::Validator, state, input, time)
    @assert time >= 1
    is_feasible = true
    state_min, state_max = TGB.state_bounds(validator.dynamics)
    control_min, control_max = TGB.control_bounds(validator.dynamics)
    environment_constraints = TGB.get_constraints(validator.environment)

    if time === 1
        if norm(validator.initial_state - state) >= validator.tolerance
            validator.verbose && @warn "Initial state is not consistent with simulated trajectory"
            is_feasible = false
        end
    else
        if norm(
            validator.dynamics(validator.previous_state, validator.previous_input, time - 1) -
            state,
        ) >= validator.tolerance
            validator.verbose && @warn "Dynamics are not consistent at time $time"
            is_feasible = false
        end
    end

    if !all(state_min .- validator.tolerance .<= state .<= state_max .+ validator.tolerance)
        validator.verbose && @warn "State at time $time is out of bounds."
        is_feasible = false
    end

    if !all(control_min .- validator.tolerance .<= input .<= control_max .+ validator.tolerance)
        validator.verbose && @warn "Input at time $time is out of bounds."
        is_feasible = false
    end

    if !all(environment_constraints(state) .>= -validator.tolerance)
        validator.verbose && @warn "State at time $time is not in the environment."
        is_feasible = false
    end

    if time > 1
        if norm(
            validator.dynamics(validator.previous_state, validator.previous_input, time - 1) -
            state,
        ) >= validator.tolerance
            validator.verbose && @warn "Dynamics are not consistent at time $time"
            is_feasible = false
        end
    end

    validator.previous_state .= state
    validator.previous_input .= input

    is_feasible
end
